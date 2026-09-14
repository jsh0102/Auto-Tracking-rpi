#include "camera_direct.hpp"

#include <sys/mman.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include <libcamera/libcamera.h>
#include <opencv2/imgproc.hpp>

#include "log.hpp"

namespace {

constexpr const char* TAG = "camera";

using namespace libcamera;

std::string shellQuote(const std::string& s) {
    std::string out = "'";
    for (char c : s) {
        if (c == '\'') out += "'\\''";
        else out += c;
    }
    out += "'";
    return out;
}

// ---------------------------------------------------------------------------
// ffmpeg 을 자식 프로세스로 띄우고 raw YUV420 을 밀어 넣는다.
// ffmpeg 은 하드웨어 인코더(h264_v4l2m2m)로 압축만 하고 RTSP 로 보낸다.
// 디코딩은 어디에도 없다 — 그게 이 구조의 핵심이다.
class Encoder {
public:
    bool start(const Config& cfg, Size size) {
        if (!cfg.stream.enabled) return true;

        std::ostringstream c;
        c << "ffmpeg -hide_banner -loglevel " << shellQuote(cfg.stream.ffmpeg_loglevel)
          << " -f rawvideo -pix_fmt yuv420p"
          << " -s " << size.width << "x" << size.height
          << " -r " << cfg.camera.fps
          << " -i pipe:0"
          << " -c:v h264_v4l2m2m"                 // 파이의 인코더 칩
          << " -b:v " << cfg.camera.bitrate
          << " -g " << cfg.camera.fps             // 1초마다 키프레임
          << " -bf 0"                             // B프레임 없음 = 지연 감소
          << " -rtsp_transport " << shellQuote(cfg.stream.rtsp_transport)
          << " -f rtsp " << shellQuote(cfg.stream.rtsp_url);
        const std::string cmd = c.str();
        LOG_I(TAG, "인코더: %s", cmd.c_str());

        int fds[2];
        if (::pipe(fds) != 0) {
            LOG_E(TAG, "pipe() 실패: %s", std::strerror(errno));
            return false;
        }
        pid_ = ::fork();
        if (pid_ < 0) {
            LOG_E(TAG, "fork() 실패: %s", std::strerror(errno));
            ::close(fds[0]); ::close(fds[1]);
            return false;
        }
        if (pid_ == 0) {
            ::setpgid(0, 0);
            ::dup2(fds[0], STDIN_FILENO);
            ::close(fds[0]); ::close(fds[1]);
            ::execl("/bin/sh", "sh", "-c", cmd.c_str(), nullptr);
            _exit(127);
        }
        ::close(fds[0]);
        fd_ = fds[1];
        ::setpgid(pid_, pid_);
        return true;
    }

    // 한 프레임을 통째로 쓴다. YUV420 세 평면이 메모리에 연속이라 한 번이면 된다.
    bool write(const unsigned char* data, size_t len) {
        if (fd_ < 0) return true;
        size_t sent = 0;
        while (sent < len) {
            ssize_t n = ::write(fd_, data + sent, len - sent);
            if (n > 0) { sent += static_cast<size_t>(n); continue; }
            if (n < 0 && errno == EINTR) continue;
            return false;   // ffmpeg 이 죽었거나 파이프가 닫힘
        }
        return true;
    }

    void stop() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }   // EOF → ffmpeg 이 스스로 끝냄
        if (pid_ > 0) {
            for (int waited = 0; waited < 2000; waited += 20) {
                if (::waitpid(pid_, nullptr, WNOHANG) != 0) { pid_ = -1; return; }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            ::kill(-pid_, SIGKILL);
            ::waitpid(pid_, nullptr, 0);
            pid_ = -1;
        }
    }

    bool alive() const {
        if (pid_ <= 0) return true;   // 송출 비활성
        return ::waitpid(pid_, nullptr, WNOHANG) == 0;
    }

private:
    pid_t pid_ = -1;
    int fd_ = -1;
};

// ---------------------------------------------------------------------------

class DirectSource : public FrameSource {
public:
    explicit DirectSource(const Config& cfg) : cfg_(cfg) {}
    ~DirectSource() override { stop(); }

    bool start() override;
    bool read(cv::Mat& out, int timeout_ms) override;
    void stop() override;
    bool alive() override { return running_ && encoder_.alive(); }

    void setOverlay(const std::vector<cv::Rect>& boxes, const std::string& label) override {
        std::lock_guard<std::mutex> lock(overlay_mutex_);
        overlay_boxes_ = boxes;
        overlay_label_ = label;
    }

private:
    bool configure();
    bool allocate();
    void onRequestCompleted(Request* request);
    void encodeLoop();
    void drawOverlay(cv::Mat& y_plane) const;
    void recycle(Request* request) {
        request->reuse(Request::ReuseBuffers);
        camera_->queueRequest(request);
    }

    const Config& cfg_;

    std::unique_ptr<CameraManager> manager_;
    std::shared_ptr<Camera> camera_;
    std::unique_ptr<CameraConfiguration> config_;
    std::unique_ptr<FrameBufferAllocator> allocator_;

    Stream* main_stream_ = nullptr;    // 송출용 (YUV420)
    Stream* lores_stream_ = nullptr;   // 검출용 (NV12)
    Size main_size_;
    Size lores_size_;
    unsigned int lores_stride_ = 0;
    size_t main_bytes_ = 0;

    std::map<const FrameBuffer*, unsigned char*> mapped_;
    std::map<const FrameBuffer*, size_t> mapped_len_;
    std::vector<std::unique_ptr<Request>> requests_;

    // 검출용 슬롯 (최신 1장) — detect.fps 로 제한해서 채운다
    std::mutex det_mutex_;
    std::condition_variable det_cv_;
    cv::Mat det_frame_;
    bool det_ready_ = false;

    // 송출용 슬롯 (최신 1장) — 카메라 fps 그대로
    std::mutex enc_mutex_;
    std::condition_variable enc_cv_;
    std::vector<unsigned char> enc_frame_;
    bool enc_ready_ = false;

    std::mutex overlay_mutex_;
    std::vector<cv::Rect> overlay_boxes_;
    std::string overlay_label_;

    Encoder encoder_;
    std::thread enc_thread_;
    std::atomic<bool> running_{false};

    std::chrono::steady_clock::duration det_interval_{};
    std::chrono::steady_clock::time_point det_last_{};
};

bool DirectSource::start() {
    const int det_fps = cfg_.detect.fps > 0 ? cfg_.detect.fps : 10;
    det_interval_ = std::chrono::microseconds(1000000 / det_fps);
    det_last_ = std::chrono::steady_clock::now() - det_interval_;

    manager_ = std::make_unique<CameraManager>();
    if (manager_->start() != 0 || manager_->cameras().empty()) {
        LOG_E(TAG, "카메라를 찾지 못했습니다");
        return false;
    }
    camera_ = manager_->cameras()[0];
    if (camera_->acquire() != 0) {
        LOG_E(TAG, "카메라를 점유하지 못했습니다 — 다른 프로그램이 쓰는 중일 수 있습니다");
        return false;
    }
    LOG_I(TAG, "카메라 직접 제어: %s", camera_->id().c_str());

    if (!configure() || !allocate()) return false;
    if (!encoder_.start(cfg_, main_size_)) return false;

    camera_->requestCompleted.connect(this, &DirectSource::onRequestCompleted);

    ControlList controls(camera_->controls());
    const int64_t frame_us = 1000000 / (cfg_.camera.fps > 0 ? cfg_.camera.fps : 30);
    controls.set(controls::FrameDurationLimits, Span<const int64_t, 2>({frame_us, frame_us}));

    if (camera_->start(&controls) != 0) {
        LOG_E(TAG, "카메라 스트림을 시작하지 못했습니다");
        return false;
    }
    running_ = true;
    enc_thread_ = std::thread(&DirectSource::encodeLoop, this);

    for (auto& r : requests_) camera_->queueRequest(r.get());
    return true;
}

bool DirectSource::configure() {
    // VideoRecording = 인코더로 보낼 큰 스트림, Viewfinder = 분석용 작은 스트림
    config_ = camera_->generateConfiguration({StreamRole::VideoRecording, StreamRole::Viewfinder});
    if (!config_ || config_->size() < 2) {
        LOG_E(TAG, "두 스트림 설정을 만들지 못했습니다");
        return false;
    }
    config_->at(0).pixelFormat = formats::YUV420;   // 인코더가 바로 먹는 형식
    config_->at(0).size = Size(cfg_.camera.width, cfg_.camera.height);
    config_->at(0).bufferCount = 4;
    config_->at(1).size = Size(cfg_.detect.width, cfg_.detect.height);
    config_->at(1).bufferCount = 4;

    // validate() 가 형식을 바꿀 수 있다. 두 스트림을 함께 쓰면 ISP 제약으로
    // lores 는 NV12 가 된다 — 그래서 검출 전에 BGR 로 변환한다.
    if (config_->validate() == CameraConfiguration::Invalid) {
        LOG_E(TAG, "요청한 설정을 맞출 수 없습니다");
        return false;
    }
    if (camera_->configure(config_.get()) != 0) {
        LOG_E(TAG, "카메라 설정 적용에 실패했습니다");
        return false;
    }

    main_stream_ = config_->at(0).stream();
    lores_stream_ = config_->at(1).stream();
    main_size_ = config_->at(0).size;
    lores_size_ = config_->at(1).size;
    lores_stride_ = config_->at(1).stride;
    main_bytes_ = config_->at(0).frameSize;

    LOG_I(TAG, "송출 %ux%u %s / 검출 %ux%u %s",
          main_size_.width, main_size_.height,
          config_->at(0).pixelFormat.toString().c_str(),
          lores_size_.width, lores_size_.height,
          config_->at(1).pixelFormat.toString().c_str());
    return true;
}

bool DirectSource::allocate() {
    allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
    for (Stream* s : {main_stream_, lores_stream_}) {
        if (allocator_->allocate(s) < 0) {
            LOG_E(TAG, "프레임 버퍼 할당 실패");
            return false;
        }
        for (const std::unique_ptr<FrameBuffer>& buf : allocator_->buffers(s)) {
            // 평면이 여러 개라도 같은 fd 안에 연속으로 놓인다. 통째로 한 번 mmap.
            size_t total = 0;
            for (const FrameBuffer::Plane& p : buf->planes()) {
                total = std::max<size_t>(total, p.offset + p.length);
            }
            void* mem = mmap(nullptr, total, PROT_READ, MAP_SHARED,
                             buf->planes()[0].fd.get(), 0);
            if (mem == MAP_FAILED) {
                LOG_E(TAG, "버퍼 mmap 실패: %s", std::strerror(errno));
                return false;
            }
            mapped_[buf.get()] = static_cast<unsigned char*>(mem);
            mapped_len_[buf.get()] = total;
        }
    }

    const auto& main_bufs = allocator_->buffers(main_stream_);
    const auto& lores_bufs = allocator_->buffers(lores_stream_);
    const size_t n = std::min(main_bufs.size(), lores_bufs.size());
    for (size_t i = 0; i < n; ++i) {
        std::unique_ptr<Request> req = camera_->createRequest();
        if (!req || req->addBuffer(main_stream_, main_bufs[i].get()) != 0 ||
            req->addBuffer(lores_stream_, lores_bufs[i].get()) != 0) {
            LOG_E(TAG, "Request 생성 실패");
            return false;
        }
        requests_.push_back(std::move(req));
    }
    LOG_I(TAG, "버퍼 %zu쌍 준비", requests_.size());
    return true;
}

// ★ libcamera 내부 스레드. 여기서 오래 붙잡으면 캡처가 밀린다 —
//   복사만 하고 바로 버퍼를 돌려준다.
void DirectSource::onRequestCompleted(Request* request) {
    if (request->status() == Request::RequestCancelled) return;

    // ── 송출용: 매 프레임 ──
    if (cfg_.stream.enabled) {
        const unsigned char* src = mapped_.at(request->buffers().at(main_stream_));
        std::lock_guard<std::mutex> lock(enc_mutex_);
        enc_frame_.assign(src, src + main_bytes_);   // 늦으면 이전 프레임은 덮어써진다
        enc_ready_ = true;
        enc_cv_.notify_one();
    }

    // ── 검출용: detect.fps 로 제한 ──
    const auto now = std::chrono::steady_clock::now();
    if (now - det_last_ >= det_interval_) {
        det_last_ = now;
        const unsigned char* src = mapped_.at(request->buffers().at(lores_stream_));
        // NV12 는 Y 평면 아래에 UV 가 붙어 높이가 1.5배인 단일 Mat 으로 다룬다.
        const cv::Mat nv12(lores_size_.height * 3 / 2, lores_size_.width, CV_8UC1,
                           const_cast<unsigned char*>(src), lores_stride_);
        std::lock_guard<std::mutex> lock(det_mutex_);
        cv::cvtColor(nv12, det_frame_, cv::COLOR_YUV2BGR_NV12);
        det_ready_ = true;
        det_cv_.notify_one();
    }

    recycle(request);
}

void DirectSource::drawOverlay(cv::Mat& y) const {
    std::vector<cv::Rect> boxes;
    std::string label;
    {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(overlay_mutex_));
        boxes = overlay_boxes_;
        label = overlay_label_;
    }
    if (boxes.empty()) return;

    // 검출은 작은 프레임 좌표라 송출 해상도로 비례 변환한다.
    const double sx = static_cast<double>(main_size_.width) / lores_size_.width;
    const double sy = static_cast<double>(main_size_.height) / lores_size_.height;
    for (const cv::Rect& b : boxes) {
        const cv::Rect scaled(static_cast<int>(b.x * sx), static_cast<int>(b.y * sy),
                              static_cast<int>(b.width * sx), static_cast<int>(b.height * sy));
        // Y(밝기) 평면에만 그린다. 색은 없지만 흰 테두리로 충분히 보인다.
        cv::rectangle(y, scaled & cv::Rect(0, 0, y.cols, y.rows), cv::Scalar(255), 2);
        if (!label.empty()) {
            cv::putText(y, label, cv::Point(scaled.x, std::max(scaled.y - 8, 16)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255), 2);
        }
    }
}

void DirectSource::encodeLoop() {
    std::vector<unsigned char> frame;
    while (running_) {
        {
            std::unique_lock<std::mutex> lock(enc_mutex_);
            enc_cv_.wait_for(lock, std::chrono::milliseconds(200),
                             [this] { return enc_ready_ || !running_; });
            if (!enc_ready_) continue;
            frame.swap(enc_frame_);
            enc_ready_ = false;
        }
        if (frame.size() < main_bytes_) continue;

        // YUV420 의 맨 앞 921,600 바이트가 Y(밝기) 평면이다. 여기에만 그린다.
        cv::Mat y(main_size_.height, main_size_.width, CV_8UC1, frame.data(),
                  main_size_.width);
        drawOverlay(y);

        if (!encoder_.write(frame.data(), main_bytes_)) {
            LOG_E(TAG, "인코더로 프레임을 보내지 못했습니다 (ffmpeg 종료?)");
            break;
        }
    }
}

bool DirectSource::read(cv::Mat& out, int timeout_ms) {
    std::unique_lock<std::mutex> lock(det_mutex_);
    if (!det_ready_) {
        det_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                         [this] { return det_ready_ || !running_; });
    }
    if (!det_ready_) return false;
    out = std::move(det_frame_);
    det_ready_ = false;
    return true;
}

void DirectSource::stop() {
    if (!camera_) return;
    running_ = false;
    det_cv_.notify_all();
    enc_cv_.notify_all();
    if (enc_thread_.joinable()) enc_thread_.join();
    encoder_.stop();

    camera_->stop();
    camera_->requestCompleted.disconnect(this, &DirectSource::onRequestCompleted);

    for (auto& e : mapped_) munmap(e.second, mapped_len_.at(e.first));
    mapped_.clear();
    mapped_len_.clear();

    requests_.clear();
    if (allocator_) {
        if (main_stream_) allocator_->free(main_stream_);
        if (lores_stream_) allocator_->free(lores_stream_);
        allocator_.reset();
    }
    config_.reset();
    camera_->release();
    camera_.reset();
    if (manager_) { manager_->stop(); manager_.reset(); }
}

}  // namespace

std::unique_ptr<FrameSource> makeDirectSource(const Config& cfg) {
    return std::make_unique<DirectSource>(cfg);
}

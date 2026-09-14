#include "camera.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "camera_direct.hpp"
#include "log.hpp"

namespace {

constexpr const char* TAG = "camera";

// 셸에 넘길 인자를 작은따옴표로 감싼다. rtsp_url 등 설정에서 온 문자열이
// 셸 메타문자를 품고 있어도 그대로 한 인자로 전달되게 하기 위한 것.
std::string shellQuote(const std::string& s) {
    std::string out = "'";
    for (char c : s) {
        if (c == '\'') out += "'\\''";
        else out += c;
    }
    out += "'";
    return out;
}

// fd 에서 정확히 n 바이트를 읽는다. EOF 나 오류면 false.
bool readFull(int fd, unsigned char* dst, size_t n) {
    size_t got = 0;
    while (got < n) {
        ssize_t r = ::read(fd, dst + got, n - got);
        if (r > 0) {
            got += static_cast<size_t>(r);
        } else if (r == 0) {
            return false;                   // 상대가 파이프를 닫음
        } else if (errno == EINTR) {
            continue;                       // 시그널에 끊긴 것뿐 — 다시
        } else {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------

class LibcameraSource : public FrameSource {
public:
    explicit LibcameraSource(const Config& cfg)
        : cfg_(cfg),
          frame_bytes_(static_cast<size_t>(cfg.detect.width) * cfg.detect.height * 3) {}

    ~LibcameraSource() override { stop(); }

    bool start() override {
        const std::string cmd = buildPipeline();
        LOG_I(TAG, "파이프라인: %s", cmd.c_str());

        int fds[2];
        if (::pipe(fds) != 0) {
            LOG_E(TAG, "pipe() 실패: %s", std::strerror(errno));
            return false;
        }

        pid_t pid = ::fork();
        if (pid < 0) {
            LOG_E(TAG, "fork() 실패: %s", std::strerror(errno));
            ::close(fds[0]);
            ::close(fds[1]);
            return false;
        }

        if (pid == 0) {
            // --- 자식 ---
            // 셸이 libcamera-vid 와 ffmpeg 두 개를 띄우므로, 새 프로세스 그룹을
            // 만들어 두고 나중에 그룹 전체에 시그널을 보낸다. 이게 없으면 셸만
            // 죽고 ffmpeg/libcamera-vid 가 카메라를 쥔 채 남는다.
            ::setpgid(0, 0);
            ::dup2(fds[1], STDOUT_FILENO);
            ::close(fds[0]);
            ::close(fds[1]);
            ::execl("/bin/sh", "sh", "-c", cmd.c_str(), nullptr);
            _exit(127);  // execl 이 돌아왔다면 실패
        }

        // --- 부모 ---
        ::close(fds[1]);
        child_ = pid;
        fd_ = fds[0];
        ::setpgid(pid, pid);  // 자식보다 부모가 먼저 도달할 수 있어 양쪽에서 호출

        stopping_ = false;
        closed_ = false;
        reader_ = std::thread(&LibcameraSource::readLoop, this);
        return true;
    }

    bool read(cv::Mat& out, int timeout_ms) override {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!has_frame_ && !closed_) {
            cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                         [this] { return has_frame_ || closed_; });
        }
        if (!has_frame_) return false;
        out = std::move(latest_);
        has_frame_ = false;
        return true;
    }

    bool alive() override {
        if (child_ <= 0) return false;
        int status = 0;
        return ::waitpid(child_, &status, WNOHANG) == 0;
    }

    void stop() override {
        if (child_ <= 0 && !reader_.joinable()) return;
        stopping_ = true;

        // 자식 그룹을 먼저 정리한다. 그래야 파이프가 닫히면서 readLoop 의
        // read() 가 0 을 돌려주고 스레드가 스스로 빠져나온다.
        if (child_ > 0) {
            ::kill(-child_, SIGTERM);
            if (!waitChild(3000)) {
                LOG_W(TAG, "파이프라인이 SIGTERM 에 응답하지 않아 강제 종료합니다");
                ::kill(-child_, SIGKILL);
                waitChild(1000);
            }
            child_ = -1;
        }
        if (reader_.joinable()) reader_.join();
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        markClosed();
    }

private:
    std::string buildPipeline() const {
        const auto& cam = cfg_.camera;
        const auto& st = cfg_.stream;
        const auto& det = cfg_.detect;

        std::ostringstream c;
        c << "libcamera-vid"
          << " --nopreview"
          << " --verbose 0"          // 0=무출력. 로그는 우리 쪽으로 통일
          << " --timeout 0"          // 무한
          << " --codec h264"
          << " --inline"             // 매 IDR 앞에 SPS/PPS 삽입 (중간 접속자용, RTSP 필수)
          << " --flush"
          << " --width " << cam.width
          << " --height " << cam.height
          << " --framerate " << cam.fps
          << " --bitrate " << cam.bitrate;
        if (cam.rotation != 0) c << " --rotation " << cam.rotation;
        if (cam.hflip) c << " --hflip";
        if (cam.vflip) c << " --vflip";
        c << " --output -";

        c << " | ffmpeg -hide_banner -loglevel " << shellQuote(st.ffmpeg_loglevel)
          << " -fflags nobuffer -flags low_delay"
          << " -f h264 -framerate " << cam.fps << " -i pipe:0";

        if (st.enabled) {
            // 재인코딩 없이 그대로 RTSP 로 밀어넣는다(-c:v copy) — CPU 를 쓰지 않는다.
            //
            // NOTE: MediaMTX 가 "RTP packets are too big (1460 > 1440)" 경고를 낸다.
            // ffmpeg 의 RTSP muxer 가 RTP 패킷 크기를 내부적으로 고정하고 있어
            // -pkt_size / -packetsize 로는 바꿀 수 없다(4.3 에서 실측 확인).
            // 서버가 알아서 잘게 쪼개 주고 비용도 무시할 수준이라 그대로 둔다.
            c << " -map 0:v -c:v copy"
              << " -rtsp_transport " << shellQuote(st.rtsp_transport)
              << " -f rtsp " << shellQuote(st.rtsp_url);
        }
        // 검출용 저해상도 raw BGR — 이게 우리 stdin 으로 들어온다.
        c << " -map 0:v -vf scale=" << det.width << ":" << det.height
          << " -r " << det.fps
          << " -pix_fmt bgr24 -f rawvideo pipe:1";
        return c.str();
    }

    void readLoop() {
        const int h = cfg_.detect.height;
        const int w = cfg_.detect.width;
        while (!stopping_) {
            // 프레임마다 새 Mat 에 직접 읽어 넣는다. Mat 은 참조 계수 방식이라
            // 아래에서 슬롯으로 옮기는 비용이 사실상 0 이다(복사 없음).
            cv::Mat frame(h, w, CV_8UC3);
            if (!readFull(fd_, frame.data, frame_bytes_)) break;

            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = std::move(frame);   // 읽는 쪽이 느리면 이전 프레임은 버려진다
            has_frame_ = true;
            cv_.notify_one();
        }
        if (!stopping_) {
            LOG_E(TAG, "프레임 스트림이 끊겼습니다 (libcamera-vid/ffmpeg 종료).");
        }
        markClosed();
    }

    void markClosed() {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
        cv_.notify_all();
    }

    // 자식이 끝날 때까지 최대 timeout_ms 기다린다.
    bool waitChild(int timeout_ms) {
        for (int waited = 0; waited < timeout_ms; waited += 20) {
            int status = 0;
            pid_t r = ::waitpid(child_, &status, WNOHANG);
            if (r == child_ || (r < 0 && errno == ECHILD)) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return false;
    }

    const Config& cfg_;
    const size_t frame_bytes_;

    pid_t child_ = -1;
    int fd_ = -1;
    std::thread reader_;
    std::atomic<bool> stopping_{false};

    std::mutex mutex_;
    std::condition_variable cv_;
    cv::Mat latest_;
    bool has_frame_ = false;
    bool closed_ = false;
};

}  // namespace

std::unique_ptr<FrameSource> makeFrameSource(const Config& cfg) {
    if (cfg.camera.backend == "libcamera") return std::make_unique<LibcameraSource>(cfg);
    if (cfg.camera.backend == "direct") return makeDirectSource(cfg);
    throw std::runtime_error("알 수 없는 camera.backend: " + cfg.camera.backend);
}

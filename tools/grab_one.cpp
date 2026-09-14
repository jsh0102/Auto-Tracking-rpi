// 2-1: libcamera 로 직접 프레임 한 장 받아 JPEG 으로 저장한다.
//
// libcamera-vid 도 ffmpeg 도 쓰지 않는다. 카메라를 우리가 직접 연다.
// 순서는 항상 이렇다:
//
//   CameraManager 시작 → 카메라 고르기 → acquire(독점) → 설정 →
//   버퍼 할당 → Request 만들기 → 콜백 등록 → start → queue → 기다림

#include <libcamera/libcamera.h>

#include <sys/mman.h>

#include <condition_variable>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace libcamera;

namespace {

std::mutex g_mutex;
std::condition_variable g_cv;
Request* g_done = nullptr;   // 완료된 Request. 콜백이 채워 준다.

// ★ 카메라가 우리를 부르는 지점. libcamera 내부 스레드에서 실행된다.
//   여기서 무거운 일을 하면 캡처가 밀리므로, 알리기만 하고 빠져나온다.
void requestComplete(Request* request) {
    if (request->status() == Request::RequestCancelled) return;
    std::lock_guard<std::mutex> lock(g_mutex);
    g_done = request;
    g_cv.notify_one();
}

}  // namespace

int main() {
    // ── ① 카메라 관리자 시작 ───────────────────────────────────────
    CameraManager cm;
    if (cm.start() != 0) {
        std::cerr << "CameraManager 시작 실패\n";
        return 1;
    }
    if (cm.cameras().empty()) {
        std::cerr << "카메라를 찾지 못했습니다\n";
        return 1;
    }

    std::shared_ptr<Camera> camera = cm.cameras()[0];
    std::cout << "카메라: " << camera->id() << "\n";

    // ── ② 독점 사용권 획득 ────────────────────────────────────────
    // 다른 프로그램(libcamera-vid 등)이 쓰고 있으면 여기서 실패한다.
    if (camera->acquire() != 0) {
        std::cerr << "카메라를 잡지 못했습니다 (다른 프로그램이 쓰는 중?)\n";
        return 1;
    }

    // ── ③ 원하는 형식 요청 ────────────────────────────────────────
    // Viewfinder = "화면에 보여줄 용도" — 적당한 해상도로 알아서 잡아 준다.
    std::unique_ptr<CameraConfiguration> config =
        camera->generateConfiguration({StreamRole::Viewfinder});
    StreamConfiguration& stream_cfg = config->at(0);
    stream_cfg.pixelFormat = formats::RGB888;
    stream_cfg.size = Size(640, 480);

    // validate() 는 "그 요청 그대로는 안 되니 가장 가까운 걸로 고쳐 줄게" 단계다.
    // 카메라가 지원하지 않는 조합을 조용히 통과시키지 않는다.
    if (config->validate() == CameraConfiguration::Invalid) {
        std::cerr << "설정을 맞출 수 없습니다\n";
        return 1;
    }
    std::cout << "설정: " << stream_cfg.toString()
              << "  stride=" << stream_cfg.stride << "\n";
    camera->configure(config.get());

    // ── ④ 사진이 담길 빈 상자(버퍼) 준비 ──────────────────────────
    // unique_ptr 로 잡는다 — 정리 시점을 우리가 정하기 위해서.
    // FrameBufferAllocator 는 내부에 shared_ptr<Camera> 를 들고 있어서,
    // 이 객체가 살아 있는 한 카메라도 해제되지 않는다.
    auto allocator = std::make_unique<FrameBufferAllocator>(camera);
    Stream* stream = stream_cfg.stream();
    if (allocator->allocate(stream) < 0) {
        std::cerr << "버퍼 할당 실패\n";
        return 1;
    }
    const auto& buffers = allocator->buffers(stream);
    std::cout << "버퍼 " << buffers.size() << "개 준비됨\n";

    // ── ⑤ Request = "이 상자에 채워 줘" 라는 주문서 ───────────────
    std::unique_ptr<Request> request = camera->createRequest();
    request->addBuffer(stream, buffers[0].get());

    // ── ⑥ 콜백 등록 → 시작 → 주문서 제출 ─────────────────────────
    camera->requestCompleted.connect(requestComplete);
    camera->start();
    camera->queueRequest(request.get());

    // ── ⑦ 카메라가 부를 때까지 대기 ───────────────────────────────
    {
        std::unique_lock<std::mutex> lock(g_mutex);
        if (!g_cv.wait_for(lock, std::chrono::seconds(5),
                           [] { return g_done != nullptr; })) {
            std::cerr << "5초 안에 프레임이 오지 않았습니다\n";
            return 1;
        }
    }

    // ── ⑧ 상자에서 픽셀 꺼내기 ────────────────────────────────────
    FrameBuffer* buffer = g_done->buffers().at(stream);
    const FrameBuffer::Plane& plane = buffer->planes()[0];

    // 버퍼는 커널/드라이버 메모리다. mmap 으로 우리 주소 공간에 붙여야 읽을 수 있다.
    void* mem = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED,
                     plane.fd.get(), plane.offset);
    if (mem == MAP_FAILED) {
        std::cerr << "mmap 실패\n";
        return 1;
    }

    // stride(한 줄의 실제 바이트 수)가 width*3 보다 클 수 있다 —
    // 하드웨어가 줄 끝을 정렬 경계까지 패딩하기 때문. cv::Mat 에 알려 줘야 한다.
    cv::Mat frame(stream_cfg.size.height, stream_cfg.size.width, CV_8UC3,
                  mem, stream_cfg.stride);
    cv::imwrite("grab_one.jpg", frame);
    std::cout << "저장: grab_one.jpg  ("
              << frame.cols << "x" << frame.rows << ")\n";

    munmap(mem, plane.length);

    // ── ⑨ 정리 ────────────────────────────────────────────────────
    // 순서가 중요하다. CameraManager 는 카메라를 쥔 것이 하나도 없을 때
    // 멈춰야 한다. allocator 와 camera(shared_ptr)가 살아 있는 채로 cm.stop()
    // 을 부르면 "Removing media device while still in use" 에러가 난다.
    // 카메라를 붙잡고 있는 것들을 전부 놓은 뒤에야 cm.stop() 을 부를 수 있다.
    camera->stop();
    allocator->free(stream);
    request.reset();     // Request 가 버퍼를 참조한다
    allocator.reset();   // allocator 가 shared_ptr<Camera> 를 들고 있다
    config.reset();
    camera->release();
    camera.reset();
    cm.stop();
    return 0;
}

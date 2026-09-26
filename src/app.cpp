#include "app.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <thread>

#include "log.hpp"

namespace {
constexpr const char* TAG = "app";
}

App::App(const Config& cfg)
    : cfg_(cfg),
      source_(makeFrameSource(cfg)),
      detector_(makeDetector(cfg)),
      motor_(makeMotor(cfg)),
      tracker_(cfg, *motor_) {}

int App::run(const volatile std::sig_atomic_t& stop) {
    LOG_I(TAG, "시작: detect=%s motor=%s rtsp=%s",
          cfg_.detect.backend.c_str(), cfg_.motor.backend.c_str(),
          cfg_.stream.enabled ? cfg_.stream.rtsp_url.c_str() : "disabled");

    if (!source_->start()) {
        LOG_E(TAG, "카메라를 시작하지 못했습니다.");
        return 1;
    }

    last_stat_ = std::chrono::steady_clock::now();
    last_snapshot_ = last_stat_;

    cv::Mat frame;
    long tick = 0;
    const int interval = cfg_.detect.interval > 0 ? cfg_.detect.interval : 1;

    while (stop == 0) {
        if (!source_->read(frame, 2000)) {
            if (!source_->alive()) {
                LOG_E(TAG, "캡처 파이프라인이 죽었습니다. 종료합니다.");
                break;
            }
            continue;
        }

        ++frames_;
        ++tick;

        if (tick % interval == 0) {
            const std::vector<Detection> dets = detector_->detect(frame);
            Detection picked;
            const Detection* prev = has_target_ ? &target_ : nullptr;
            if (selectTarget(dets, prev, frame.size(), picked)) {
                target_ = picked;
                has_target_ = true;
            } else {
                has_target_ = false;
            }

            // 송출 영상에 박스를 그리도록 캡처 쪽에 알려 준다.
            // 송출을 직접 하지 않는 백엔드에서는 아무 일도 일어나지 않는다.
            std::vector<cv::Rect> boxes;
            boxes.reserve(dets.size());
            for (const Detection& d : dets) boxes.push_back(d.box);
            source_->setOverlay(boxes, boxes.empty() ? "" : "Person");
        }

        // 하드웨어 비상정지가 걸려 있으면 모터 제어만 건너뛴다.
        // 캡처·검출·송출은 계속 돈다 — 영상은 나가야 한다.
        //
        // 명령을 보내지 않는 것이 중요하다. 보내면 커널이 거부해 로그가 쏟아지고,
        // 그보다 나쁘게는 motor 쪽 "현재 각도" 기록만 앞서 나가 해제하는 순간
        // 서보가 그 차이만큼 튄다. 안 보내면 기록이 멈춰 있어 그 문제가 없다.
        const bool estopped = motor_->emergencyStopped();
        if (estopped != estop_prev_) {
            LOG_W(TAG, estopped ? "비상정지 — 추적 중단" : "비상정지 해제 — 추적 재개");
            tracker_.reset();
            estop_prev_ = estopped;
        }

        double dpan = 0.0;
        double dtilt = 0.0;
        if (!estopped)
            tracker_.update(has_target_ ? &target_ : nullptr, frame.size(), dpan, dtilt);

        reportStats();
        maybeSnapshot(frame);
    }

    LOG_I(TAG, "정리 중...");
    source_->stop();
    motor_->home();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));  // 중립까지 갈 시간
    motor_->release();
    LOG_I(TAG, "종료 완료");
    return 0;
}

void App::reportStats() {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_stat_).count();
    if (elapsed < 5.0) return;

    double pan = 0;
    double tilt = 0;
    motor_->position(pan, tilt);
    LOG_I(TAG, "%.1f fps | 타겟 %s | pan=%.1f tilt=%.1f",
          frames_ / elapsed, has_target_ ? "있음" : "없음", pan, tilt);
    frames_ = 0;
    last_stat_ = now;
}

void App::maybeSnapshot(const cv::Mat& frame) {
    if (cfg_.debug.snapshot_path.empty()) return;
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_snapshot_).count() <
        cfg_.debug.snapshot_interval) {
        return;
    }
    last_snapshot_ = now;
    cv::imwrite(cfg_.debug.snapshot_path, annotate(frame));
}

cv::Mat App::annotate(const cv::Mat& frame) const {
    cv::Mat out = frame.clone();
    const int w = out.cols;
    const int h = out.rows;

    cv::drawMarker(out, cv::Point(w / 2, h / 2), cv::Scalar(0, 255, 255),
                   cv::MARKER_CROSS, 16, 1);
    const double dz = cfg_.track.deadzone;
    cv::rectangle(out,
                  cv::Point(static_cast<int>(w / 2.0 * (1 - dz)),
                            static_cast<int>(h / 2.0 * (1 - dz))),
                  cv::Point(static_cast<int>(w / 2.0 * (1 + dz)),
                            static_cast<int>(h / 2.0 * (1 + dz))),
                  cv::Scalar(80, 80, 80), 1);

    if (has_target_) {
        cv::rectangle(out, target_.box, cv::Scalar(0, 255, 0), 2);
        char label[32];
        std::snprintf(label, sizeof(label), "%.2f", target_.score);
        cv::putText(out, label, cv::Point(target_.box.x, std::max(target_.box.y - 5, 10)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1);
    }

    double pan = 0;
    double tilt = 0;
    motor_->position(pan, tilt);
    char status[64];
    std::snprintf(status, sizeof(status), "pan %+.1f tilt %+.1f", pan, tilt);
    cv::putText(out, status, cv::Point(5, h - 8), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(255, 255, 255), 1);
    return out;
}

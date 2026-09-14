#pragma once

#include <chrono>
#include <csignal>
#include <memory>

#include <opencv2/core.hpp>

#include "camera.hpp"
#include "config.hpp"
#include "detector.hpp"
#include "motor.hpp"
#include "tracker.hpp"

// 캡처 -> 검출 -> 타겟 선택 -> 팬/틸트 제어 메인 루프.
class App {
public:
    explicit App(const Config& cfg);

    // stop 이 0 이 아니게 될 때까지 돈다. 종료 코드를 반환.
    int run(const volatile std::sig_atomic_t& stop);

private:
    void reportStats();
    void maybeSnapshot(const cv::Mat& frame);
    cv::Mat annotate(const cv::Mat& frame) const;

    const Config& cfg_;
    // 선언 순서 = 생성 순서. tracker_ 가 motor_ 를 참조하므로 motor_ 가 먼저다.
    std::unique_ptr<FrameSource> source_;
    std::unique_ptr<Detector> detector_;
    std::unique_ptr<PanTilt> motor_;
    PanTiltTracker tracker_;

    Detection target_{};
    bool has_target_ = false;

    long frames_ = 0;
    std::chrono::steady_clock::time_point last_stat_{};
    std::chrono::steady_clock::time_point last_snapshot_{};
};

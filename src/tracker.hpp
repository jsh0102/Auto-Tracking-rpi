#pragma once

#include <chrono>

#include <opencv2/core.hpp>

#include "config.hpp"
#include "detector.hpp"
#include "motor.hpp"

// 정규화 오차(-1..1)를 받아 각도 변화량(deg)을 내는 PID.
class PID {
public:
    explicit PID(const PIDConfig& cfg) : cfg_(cfg) {}

    void reset();
    double update(double error);

private:
    PIDConfig cfg_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    bool has_prev_ = false;
    std::chrono::steady_clock::time_point prev_time_{};
    bool has_time_ = false;
};

// 타겟 박스를 화면 중앙에 유지하도록 서보를 움직인다.
class PanTiltTracker {
public:
    PanTiltTracker(const Config& cfg, PanTilt& motor)
        : cfg_(cfg), motor_(motor),
          pan_pid_(cfg.track.pan_pid), tilt_pid_(cfg.track.tilt_pid) {}

    // 한 틱 진행. target 이 nullptr 이면 소실 처리.
    // 이번 틱에 적용한 이동량을 dpan/dtilt 로 돌려준다.
    void update(const Detection* target, cv::Size frame_size, double& dpan, double& dtilt);

private:
    void onLost();

    const Config& cfg_;
    PanTilt& motor_;
    PID pan_pid_;
    PID tilt_pid_;
    std::chrono::steady_clock::time_point last_seen_{};
    bool has_last_seen_ = false;
    bool homed_ = true;
};

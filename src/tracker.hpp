#pragma once

#include <chrono>
#include <deque>

#include <opencv2/core.hpp>

#include "config.hpp"
#include "detector.hpp"
#include "motor.hpp"

// 정규화 오차(-1..1)를 받아 각도 변화량(deg)을 내는 PID.
class PID {
public:
    explicit PID(const PIDConfig& cfg) : cfg_(cfg) {}

    void reset();

    // error    : 비례·적분이 쓰는 값. 지연 보상을 거친 "현재 상태 추정치".
    // measured : 미분이 쓰는 값. 보상 전 화면에서 실제로 읽은 값.
    //
    // 미분을 보정된 오차로 계산하면, 보상이 오차를 깎은 것을 "타겟이 빠르게
    // 다가온다" 로 오해해 반대로 제동한다(실측에서 ±6도 왕복 확인).
    // 화면이 안 변했으면 변화 속도는 0 이다 — 미분은 화면만 봐야 한다.
    double update(double error, double measured);

private:
    PIDConfig cfg_;
    double integral_ = 0.0;
    double prev_measured_ = 0.0;
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

    // 제어를 한동안 멈췄다가 다시 시작할 때 부른다(비상정지 해제 등).
    // 멈춘 동안 쌓인 PID 이력과 지연 보상 창은 지금 상황과 무관하므로 버린다.
    void reset();

private:
    void onLost();

    // 아직 영상에 나타나지 않은 명령 한 건.
    struct Pending {
        std::chrono::steady_clock::time_point at;
        double pan;
        double tilt;
    };

    // latency_ms 안에 내려진 명령의 합. 이것이 "영상이 아직 모르는 이동량" 이다.
    void inFlight(std::chrono::steady_clock::time_point now,
                  double& pan, double& tilt);

    const Config& cfg_;
    PanTilt& motor_;
    PID pan_pid_;
    PID tilt_pid_;
    std::chrono::steady_clock::time_point last_seen_{};
    bool has_last_seen_ = false;
    bool homed_ = true;

    std::deque<Pending> pending_;
};

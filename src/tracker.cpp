#include "tracker.hpp"

#include <cmath>

#include "log.hpp"

namespace {

constexpr const char* TAG = "tracker";

double clampValue(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

void PID::reset() {
    integral_ = 0.0;
    has_prev_ = false;
    has_time_ = false;
}

double PID::update(double error) {
    const auto now = std::chrono::steady_clock::now();
    double dt = 0.0;
    if (has_time_) {
        dt = std::chrono::duration<double>(now - prev_time_).count();
        if (dt < 1e-3) dt = 1e-3;
    }
    prev_time_ = now;
    has_time_ = true;

    double out = cfg_.kp * error;

    if (dt > 0.0) {
        integral_ = clampValue(integral_ + error * dt, -cfg_.integral_limit,
                               cfg_.integral_limit);
        out += cfg_.ki * integral_;
        if (has_prev_) {
            out += cfg_.kd * (error - prev_error_) / dt;
        }
    }

    prev_error_ = error;
    has_prev_ = true;
    return clampValue(out, -cfg_.max_step, cfg_.max_step);
}

void PanTiltTracker::update(const Detection* target, cv::Size frame_size,
                            double& dpan, double& dtilt) {
    dpan = 0.0;
    dtilt = 0.0;
    if (!cfg_.track.enabled) return;
    if (target == nullptr) {
        onLost();
        return;
    }

    const cv::Point2f c = target->center();
    const double half_w = frame_size.width / 2.0;
    const double half_h = frame_size.height / 2.0;

    // 정규화 오차를 "모터가 움직여야 할 방향" 부호로 맞춘다.
    //   pan  + : 타겟이 화면 오른쪽 -> 오른쪽으로 팬
    //   tilt + : 타겟이 화면 위쪽   -> 위로 틸트 (화면 y축과 반대)
    const double err_x = (c.x - half_w) / half_w;
    const double err_y = (half_h - c.y) / half_h;

    last_seen_ = std::chrono::steady_clock::now();
    has_last_seen_ = true;
    homed_ = false;

    const double dz = cfg_.track.deadzone;
    dpan = (std::fabs(err_x) < dz) ? 0.0 : pan_pid_.update(err_x);
    dtilt = (std::fabs(err_y) < dz) ? 0.0 : tilt_pid_.update(err_y);

    if (dpan != 0.0 || dtilt != 0.0) {
        motor_.moveBy(dpan, dtilt);
    }
}

void PanTiltTracker::onLost() {
    pan_pid_.reset();
    tilt_pid_.reset();
    if (homed_ || !cfg_.track.recenter_on_lost || !has_last_seen_) return;

    const double idle =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_seen_).count();
    if (idle < cfg_.track.lost_timeout) return;

    LOG_I(TAG, "타겟 소실 — 홈 위치로 복귀");
    motor_.home();
    homed_ = true;
}

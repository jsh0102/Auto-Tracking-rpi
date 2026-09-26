#include "tracker.hpp"

#include <algorithm>
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

double PID::update(double error, double measured) {
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
            // 보정된 오차가 아니라 화면에서 읽은 값의 변화로 미분한다.
            out += cfg_.kd * (measured - prev_measured_) / dt;
        }
    }

    prev_measured_ = measured;
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
    double err_x = (c.x - half_w) / half_w;
    double err_y = (half_h - c.y) / half_h;

    last_seen_ = std::chrono::steady_clock::now();
    has_last_seen_ = true;
    homed_ = false;

    // ── 지연 보상 ──────────────────────────────────────────────
    // 검출·추적 결과는 과거 장면이라 방금 보낸 명령이 반영돼 있지 않다.
    // 이미 명령한 이동량이 만들 오차 변화를 미리 빼주지 않으면, 같은 오차로
    // 여러 번 명령해 크게 지나친다(실측에서 ±20도 진동).
    const double raw_x = err_x;   // 보상 전. 미분이 쓸 값
    const double raw_y = err_y;

    double ff_pan = 0.0;
    double ff_tilt = 0.0;
    if (cfg_.track.compensate_latency) {
        inFlight(std::chrono::steady_clock::now(), ff_pan, ff_tilt);
        err_x -= ff_pan * cfg_.track.pan_deg_to_err;
        err_y -= ff_tilt * cfg_.track.tilt_deg_to_err;
    }

    const double dz = cfg_.track.deadzone;
    dpan = (std::fabs(err_x) < dz) ? 0.0 : pan_pid_.update(err_x, raw_x);
    dtilt = (std::fabs(err_y) < dz) ? 0.0 : tilt_pid_.update(err_y, raw_y);

    // 판단 근거를 그대로 남긴다. 방향 문제를 추측으로 좁히지 않기 위해서다.
    //   box    : 검출된 사람의 화면상 중심 (프레임 크기 대비)
    //   err    : 중심에서 얼마나 벗어났나 (-1 왼쪽/아래 … +1 오른쪽/위)
    //   d      : 이번 틱에 명령한 이동량(도)
    //   now    : 명령 후 서보 각도
    if (logEnabled(LogLevel::Debug)) {
        double pan = 0;
        double tilt = 0;
        motor_.position(pan, tilt);
        LOG_D(TAG,
              "box=(%.0f,%.0f)/%dx%d  err=(%+.2f,%+.2f)  ff=(%+.1f,%+.1f)  "
              "d=(%+.1f,%+.1f)  now=(%+.1f,%+.1f)",
              c.x, c.y, frame_size.width, frame_size.height,
              err_x, err_y, ff_pan, ff_tilt, dpan, dtilt, pan, tilt);
    }

    if (dpan != 0.0 || dtilt != 0.0) {
        motor_.moveBy(dpan, dtilt);
        // 방금 내린 명령을 "아직 영상에 없는 것" 으로 기록한다.
        pending_.push_back({std::chrono::steady_clock::now(), dpan, dtilt});
    }
}

void PanTiltTracker::inFlight(std::chrono::steady_clock::time_point now,
                             double& pan, double& tilt) {
    const double horizon_ms = cfg_.track.latency_ms;
    const auto horizon = std::chrono::milliseconds(static_cast<long>(horizon_ms));

    // 지연 시간이 지난 명령은 이미 영상에 반영됐다고 본다.
    while (!pending_.empty() && now - pending_.front().at > horizon) {
        pending_.pop_front();
    }

    // 나이에 따라 가중치를 1 → 0 으로 선형 감소시킨다.
    //
    // 명령이 영상에 나타나는 시점을 정확히는 모른다(프레임 타이밍, 검출/추적 경로에
    // 따라 달라짐). 가중치 없이 "창 안이면 100%, 밖이면 0%" 로 두면 명령이 창을
    // 벗어나는 순간 보상량이 뚝 떨어지고, 보정 오차가 그만큼 튄다.
    // 실측에서 ff 가 -6 에서 +6 으로 한 번에 뒤집혀 오차가 0.5 튀는 것을 확인했다.
    // 불확실한 시점을 한 점에 몰지 않고 구간에 퍼뜨리는 셈이다.
    pan = 0.0;
    tilt = 0.0;
    for (const Pending& p : pending_) {
        const double age_ms =
            std::chrono::duration<double, std::milli>(now - p.at).count();
        const double w = std::max(0.0, 1.0 - age_ms / horizon_ms);
        pan += p.pan * w;
        tilt += p.tilt * w;
    }
}

void PanTiltTracker::reset() {
    pan_pid_.reset();
    tilt_pid_.reset();
    pending_.clear();

    // 멈춰 있던 동안은 타겟을 "못 본" 게 아니라 "안 본" 것이다.
    // 이걸 안 되돌리면 재개하는 순간 "오래 못 봤다" 로 판정해 홈으로 크게 돈다.
    last_seen_ = std::chrono::steady_clock::now();
}

void PanTiltTracker::onLost() {
    pan_pid_.reset();
    tilt_pid_.reset();
    pending_.clear();
    if (homed_ || !cfg_.track.recenter_on_lost || !has_last_seen_) return;

    const double idle =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_seen_).count();
    if (idle < cfg_.track.lost_timeout) return;

    LOG_I(TAG, "타겟 소실 — 홈 위치로 복귀");
    motor_.home();
    homed_ = true;
}

#include "motor.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <thread>

#include <pigpiod_if2.h>

#include "log.hpp"

namespace {

constexpr const char* TAG = "motor";

double clampValue(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

class DummyPanTilt : public PanTilt {
public:
    explicit DummyPanTilt(const MotorConfig& cfg)
        : cfg_(cfg), pan_(cfg.pan_home), tilt_(cfg.tilt_home) {}

    void moveTo(double pan, double tilt) override {
        std::lock_guard<std::mutex> lock(mutex_);
        pan_ = clampValue(pan, cfg_.pan_min, cfg_.pan_max);
        tilt_ = clampValue(tilt, cfg_.tilt_min, cfg_.tilt_max);
        LOG_D(TAG, "dummy -> pan=%.1f tilt=%.1f", pan_, tilt_);
    }
    void position(double& pan, double& tilt) const override {
        std::lock_guard<std::mutex> lock(mutex_);
        pan = pan_;
        tilt = tilt_;
    }
    void release() override {}
    void home() override { moveTo(cfg_.pan_home, cfg_.tilt_home); }

private:
    MotorConfig cfg_;
    mutable std::mutex mutex_;
    double pan_;
    double tilt_;
};

// MG90(S) 서보 2개를 pigpiod 를 통해 구동한다.
//
// pigpio 를 직접 쓰지 않고 pigpiod_if2(데몬 클라이언트)를 쓰는 이유:
// 라이브러리를 직접 쓰면 root 권한이 필요하지만, 데몬에 붙으면 일반 사용자로
// 돌릴 수 있다. 데몬은 `sudo systemctl enable --now pigpiod` 로 띄운다.
class ServoPanTilt : public PanTilt {
public:
    explicit ServoPanTilt(const MotorConfig& cfg) : cfg_(cfg) {
        pi_ = pigpio_start(cfg_.pigpio_host.c_str(), cfg_.pigpio_port.c_str());
        if (pi_ < 0) {
            throw std::runtime_error(
                "pigpiod 에 연결하지 못했습니다 (" + cfg_.pigpio_host + ":" +
                cfg_.pigpio_port + "). sudo systemctl enable --now pigpiod");
        }
        pan_ = cfg_.pan_home;
        tilt_ = cfg_.tilt_home;
        home();
        if (cfg_.idle_detach) {
            idle_thread_ = std::thread(&ServoPanTilt::idleLoop, this);
        }
    }

    ~ServoPanTilt() override {
        stopping_ = true;
        if (idle_thread_.joinable()) idle_thread_.join();
        if (pi_ >= 0) {
            release();
            pigpio_stop(pi_);
        }
    }

    void moveTo(double pan, double tilt) override {
        std::lock_guard<std::mutex> lock(mutex_);
        pan_ = clampValue(pan, cfg_.pan_min, cfg_.pan_max);
        tilt_ = clampValue(tilt, cfg_.tilt_min, cfg_.tilt_max);
        writeServo(cfg_.pan_pin, cfg_.pan_invert ? -pan_ : pan_);
        writeServo(cfg_.tilt_pin, cfg_.tilt_invert ? -tilt_ : tilt_);
        last_move_ = std::chrono::steady_clock::now();
        detached_ = false;
    }

    void position(double& pan, double& tilt) const override {
        std::lock_guard<std::mutex> lock(mutex_);
        pan = pan_;
        tilt = tilt_;
    }

    void release() override {
        std::lock_guard<std::mutex> lock(mutex_);
        releaseLocked();
    }

    void home() override { moveTo(cfg_.pan_home, cfg_.tilt_home); }

private:
    // 각도(-90..90) -> 펄스폭(마이크로초). 호출자가 mutex_ 를 쥐고 있어야 한다.
    void writeServo(int gpio, double angle) {
        const double min_us = cfg_.min_pulse_ms * 1000.0;
        const double max_us = cfg_.max_pulse_ms * 1000.0;
        const double t = (clampValue(angle, -90.0, 90.0) + 90.0) / 180.0;
        const int pulse = static_cast<int>(std::lround(min_us + t * (max_us - min_us)));
        set_servo_pulsewidth(pi_, static_cast<unsigned>(gpio), static_cast<unsigned>(pulse));
    }

    void releaseLocked() {
        // 펄스폭 0 = 펄스 중단. 서보는 힘을 놓는다.
        set_servo_pulsewidth(pi_, static_cast<unsigned>(cfg_.pan_pin), 0);
        set_servo_pulsewidth(pi_, static_cast<unsigned>(cfg_.tilt_pin), 0);
        detached_ = true;
    }

    // 마지막 이동 후 일정 시간이 지나면 PWM 을 끊는다.
    void idleLoop() {
        using namespace std::chrono;
        while (!stopping_) {
            std::this_thread::sleep_for(milliseconds(100));
            std::lock_guard<std::mutex> lock(mutex_);
            if (detached_) continue;
            if (steady_clock::now() - last_move_ > milliseconds(600)) {
                releaseLocked();
            }
        }
    }

    MotorConfig cfg_;
    int pi_ = -1;
    mutable std::mutex mutex_;
    double pan_ = 0.0;
    double tilt_ = 0.0;
    bool detached_ = false;
    std::chrono::steady_clock::time_point last_move_{};
    std::thread idle_thread_;
    std::atomic<bool> stopping_{false};
};

}  // namespace

std::unique_ptr<PanTilt> makeMotor(const Config& cfg) {
    if (cfg.motor.backend == "dummy") {
        return std::make_unique<DummyPanTilt>(cfg.motor);
    }
    if (cfg.motor.backend != "servo") {
        throw std::runtime_error("알 수 없는 motor.backend: " + cfg.motor.backend);
    }
    try {
        return std::make_unique<ServoPanTilt>(cfg.motor);
    } catch (const std::exception& e) {
        LOG_E(TAG, "%s", e.what());
        LOG_W(TAG, "모터 없이 계속합니다 (dummy).");
        return std::make_unique<DummyPanTilt>(cfg.motor);
    }
}

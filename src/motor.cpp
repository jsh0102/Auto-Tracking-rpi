#include "motor.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <thread>

#include <fcntl.h>
#include <unistd.h>

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

// 두 백엔드의 공통부.
//
// 각도를 펄스폭(µs)으로 바꾸고, 가동 범위로 자르고, invert 를 적용하고, 멈춘 지
// 오래되면 펄스를 끊는 것까지는 pigpio 든 커널 드라이버든 완전히 같다.
// 다른 것은 "만든 펄스폭을 어디로 보내느냐" 하나뿐이라, 그 한 가지만 파생
// 클래스가 채우게 했다. 복사해 두 벌을 만들면 한쪽만 고치는 사고가 난다.
class PulseServoPanTilt : public PanTilt {
public:
    explicit PulseServoPanTilt(const MotorConfig& cfg)
        : cfg_(cfg), pan_(cfg.pan_home), tilt_(cfg.tilt_home) {}

    void moveTo(double pan, double tilt) override {
        std::lock_guard<std::mutex> lock(mutex_);
        pan_ = clampValue(pan, cfg_.pan_min, cfg_.pan_max);
        tilt_ = clampValue(tilt, cfg_.tilt_min, cfg_.tilt_max);
        writeAxis(Axis::Pan, cfg_.pan_invert ? -pan_ : pan_);
        writeAxis(Axis::Tilt, cfg_.tilt_invert ? -tilt_ : tilt_);
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

protected:
    enum class Axis { Pan, Tilt };

    // 파생 클래스가 장치를 연 "뒤에" 부른다. 생성자에서 먼저 부르면 아직
    // 만들어지지 않은 파생 부분을 idle 스레드가 건드린다.
    void startServo() {
        home();
        if (cfg_.idle_detach) {
            idle_thread_ = std::thread(&PulseServoPanTilt::idleLoop, this);
        }
    }

    // 파생 클래스 소멸자의 "첫 줄"에서 부른다. 스레드를 먼저 세우지 않으면
    // 이미 파괴된 파생 객체의 sendPulse 를 부르게 된다.
    void stopServo() {
        stopping_ = true;
        if (idle_thread_.joinable()) idle_thread_.join();
    }

    int gpioOf(Axis axis) const {
        return axis == Axis::Pan ? cfg_.pan_pin : cfg_.tilt_pin;
    }
    const char* nameOf(Axis axis) const {
        return axis == Axis::Pan ? "pan" : "tilt";
    }

    // 파생 클래스가 채우는 두 가지. 호출자가 mutex_ 를 쥐고 있다.
    virtual void sendPulse(Axis axis, int pulse_us) = 0;
    virtual void sendRelease(Axis axis) = 0;

    MotorConfig cfg_;

private:
    // 각도(-90..90) -> 펄스폭(마이크로초).
    void writeAxis(Axis axis, double angle) {
        const double min_us = cfg_.min_pulse_ms * 1000.0;
        const double max_us = cfg_.max_pulse_ms * 1000.0;
        const double t = (clampValue(angle, -90.0, 90.0) + 90.0) / 180.0;
        sendPulse(axis, static_cast<int>(std::lround(min_us + t * (max_us - min_us))));
    }

    void releaseLocked() {
        sendRelease(Axis::Pan);
        sendRelease(Axis::Tilt);
        detached_ = true;
    }

    // 마지막 이동 후 일정 시간이 지나면 PWM 을 끊는다. 지터와 발열이 준다.
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

    mutable std::mutex mutex_;
    double pan_ = 0.0;
    double tilt_ = 0.0;
    bool detached_ = false;
    std::chrono::steady_clock::time_point last_move_{};
    std::thread idle_thread_;
    std::atomic<bool> stopping_{false};
};

// backend=servo — pigpiod 를 통해 구동한다.
//
// pigpio 를 직접 쓰지 않고 pigpiod_if2(데몬 클라이언트)를 쓰는 이유:
// 라이브러리를 직접 쓰면 root 권한이 필요하지만, 데몬에 붙으면 일반 사용자로
// 돌릴 수 있다. 데몬은 `sudo systemctl enable --now pigpiod` 로 띄운다.
//
// 데몬이 /dev/mem 으로 GPIO 레지스터를 직접 만져 DMA 로 펄스를 만든다.
// 커널을 우회하므로 커널은 그 핀이 쓰이는 줄 모른다.
class ServoPanTilt : public PulseServoPanTilt {
public:
    explicit ServoPanTilt(const MotorConfig& cfg) : PulseServoPanTilt(cfg) {
        pi_ = pigpio_start(cfg_.pigpio_host.c_str(), cfg_.pigpio_port.c_str());
        if (pi_ < 0) {
            throw std::runtime_error(
                "pigpiod 에 연결하지 못했습니다 (" + cfg_.pigpio_host + ":" +
                cfg_.pigpio_port + "). sudo systemctl enable --now pigpiod");
        }
        LOG_I(TAG, "pigpiod 연결 (%s:%s), GPIO%d/%d", cfg_.pigpio_host.c_str(),
              cfg_.pigpio_port.c_str(), cfg_.pan_pin, cfg_.tilt_pin);
        startServo();
    }

    ~ServoPanTilt() override {
        stopServo();
        if (pi_ >= 0) {
            release();
            pigpio_stop(pi_);
        }
    }

protected:
    void sendPulse(Axis axis, int pulse_us) override {
        set_servo_pulsewidth(pi_, static_cast<unsigned>(gpioOf(axis)),
                             static_cast<unsigned>(pulse_us));
    }
    // 펄스폭 0 = 펄스 중단. 서보는 힘을 놓는다.
    void sendRelease(Axis axis) override {
        set_servo_pulsewidth(pi_, static_cast<unsigned>(gpioOf(axis)), 0);
    }

private:
    int pi_ = -1;
};

// backend=kservo — 커널 모듈(kernel/servo/servo.ko)에 써서 구동한다.
//
// 펄스는 커널이 hrtimer 로 만든다. 여기서는 장치 파일에 한 줄 쓰는 게 전부다.
//
//   pan 1683us     펄스폭 지정
//   pan off        펄스 중단
//
// 각도가 아니라 µs 를 보내는 이유: 서보마다 다른 보정값(min/max_pulse_ms)은
// config.json 에 있고, 커널은 그걸 몰라도 된다. 커널이 각도를 받으면 보정값이
// 바뀔 때마다 모듈을 다시 빌드해야 한다.
class KernelServoPanTilt : public PulseServoPanTilt {
public:
    explicit KernelServoPanTilt(const MotorConfig& cfg) : PulseServoPanTilt(cfg) {
        fd_ = ::open(cfg_.kservo_path.c_str(), O_WRONLY | O_CLOEXEC);
        if (fd_ < 0) {
            throw std::runtime_error(
                cfg_.kservo_path + " 를 열지 못했습니다 (" + std::strerror(errno) +
                "). 모듈이 적재됐는지(lsmod | grep servo), udev 규칙이 들어갔는지"
                " 확인하세요.");
        }
        LOG_I(TAG, "커널 드라이버 사용 (%s)", cfg_.kservo_path.c_str());
        startServo();
    }

    ~KernelServoPanTilt() override {
        stopServo();
        if (fd_ >= 0) {
            release();
            ::close(fd_);
        }
    }

protected:
    void sendPulse(Axis axis, int pulse_us) override {
        char arg[16];
        std::snprintf(arg, sizeof(arg), "%dus", pulse_us);
        sendCommand(nameOf(axis), arg);
    }
    void sendRelease(Axis axis) override { sendCommand(nameOf(axis), "off"); }

private:
    // 한 번의 write 가 한 개의 명령이다. 커널 쪽 servo_write 가 그렇게 읽는다.
    void sendCommand(const char* axis, const char* arg) {
        char line[32];
        const int n = std::snprintf(line, sizeof(line), "%s %s\n", axis, arg);
        if (n <= 0 || n >= static_cast<int>(sizeof(line))) return;

        // 커널이 값을 거부하면(-EINVAL/-ERANGE) write 가 실패한다.
        // 조용히 넘기면 "왜 안 움직이지" 로 한참을 잃는다.
        if (::write(fd_, line, static_cast<size_t>(n)) != n) {
            LOG_W(TAG, "%s 전송 실패 (%s): %s", cfg_.kservo_path.c_str(),
                  std::strerror(errno), line);
        }
    }

    int fd_ = -1;
};

}  // namespace

std::unique_ptr<PanTilt> makeMotor(const Config& cfg) {
    const std::string& backend = cfg.motor.backend;

    if (backend == "dummy") {
        return std::make_unique<DummyPanTilt>(cfg.motor);
    }
    if (backend != "servo" && backend != "kservo") {
        throw std::runtime_error("알 수 없는 motor.backend: " + backend);
    }
    try {
        if (backend == "kservo") {
            return std::make_unique<KernelServoPanTilt>(cfg.motor);
        }
        return std::make_unique<ServoPanTilt>(cfg.motor);
    } catch (const std::exception& e) {
        LOG_E(TAG, "%s", e.what());
        LOG_W(TAG, "모터 없이 계속합니다 (dummy).");
        return std::make_unique<DummyPanTilt>(cfg.motor);
    }
}

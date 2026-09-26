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
#include <string>
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
        if (!cfg_.kservo_estop.empty()) {
            estop_fd_ = ::open(cfg_.kservo_estop.c_str(), O_RDONLY | O_CLOEXEC);
            if (estop_fd_ < 0) {
                LOG_W(TAG, "비상정지 상태를 읽을 수 없습니다 (%s). 무시하고 진행합니다",
                      cfg_.kservo_estop.c_str());
            }
        }
        LOG_I(TAG, "커널 드라이버 사용 (%s)%s", cfg_.kservo_path.c_str(),
              estop_fd_ >= 0 ? ", 비상정지 연동" : "");
        startServo();
    }

    // 제어 루프마다 한 번 읽는다(초당 5회). 한 번에 3µs 남짓이라 부담이 없다.
    bool emergencyStopped() const override {
        if (estop_fd_ < 0) return false;

        char buf[128];
        const ssize_t n = ::pread(estop_fd_, buf, sizeof(buf) - 1, 0);
        if (n <= 0) return false;
        buf[n] = '\0';

        // "mode kernel  engaged 1  stops 3 ..."
        const char* p = std::strstr(buf, "engaged ");
        return p && p[8] == '1';
    }

    ~KernelServoPanTilt() override {
        stopServo();
        if (estop_fd_ >= 0) ::close(estop_fd_);
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
        //
        // 단 EBUSY 는 다르다 — 비상정지가 걸려 있어 커널이 막은 것이고,
        // 그것이 정상 동작이다. 오류로 찍으면 로그가 쏟아져 진짜 문제가 묻힌다.
        if (::write(fd_, line, static_cast<size_t>(n)) != n) {
            if (errno == EBUSY) {
                LOG_D(TAG, "비상정지 중이라 거부됨: %s", line);
            } else {
                LOG_W(TAG, "%s 전송 실패 (%s): %s", cfg_.kservo_path.c_str(),
                      std::strerror(errno), line);
            }
        }
    }

    int fd_ = -1;
    int estop_fd_ = -1;
};

// backend=syspwm — 커널 PWM 프레임워크를 sysfs 창구로 직접 구동한다.
//
// 펄스를 만드는 것이 CPU 가 아니라 BCM2711 안의 PWM 회로다. duty_cycle 을 한 번
// 써 두면 회로가 그 파형을 무한히 반복한다. CPU 가 아무리 바빠도 펄스폭이
// 흔들리지 않으므로, kservo(hrtimer)에서 부하 시 나타나던 지터가 사라진다.
//
// 전제 조건이 두 개 있다. 둘 다 /boot/config.txt 에 있고 재부팅이 필요하다.
//   dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4   핀을 PWM 회로에 붙인다
//   #dtparam=audio=on                                    오디오가 같은 채널을 쓴다
//
// ★ 이 백엔드에는 비상정지가 없다 — emergencyStopped() 는 기본값 false 를 쓴다.
//   커널 모듈이 빠진 그림이라 버튼 IRQ 를 받을 주체가 없다. 하드웨어 PWM 이
//   추적 흔들림을 얼마나 줄이는지 재기 위한 실험용 백엔드이고, 비상정지는
//   커널 모듈을 PWM 소비자로 개조하면서 되찾는다.
class SysPwmPanTilt : public PulseServoPanTilt {
public:
    explicit SysPwmPanTilt(const MotorConfig& cfg) : PulseServoPanTilt(cfg) {
        openChannel(Axis::Pan, cfg_.syspwm_pan_channel);
        openChannel(Axis::Tilt, cfg_.syspwm_tilt_channel);
        LOG_I(TAG, "하드웨어 PWM 사용 (%s, 채널 %d/%d, 주기 %dus), 비상정지 없음",
              cfg_.syspwm_chip.c_str(), cfg_.syspwm_pan_channel,
              cfg_.syspwm_tilt_channel, kPeriodUs);
        startServo();
    }

    ~SysPwmPanTilt() override {
        stopServo();
        release();
        for (int& fd : duty_fd_) {
            if (fd >= 0) ::close(fd);
            fd = -1;
        }
        // unexport 하지 않는다. 채널을 열어 둔 채 duty=0 이면 핀은 LOW 로
        // 붙잡혀 있고, 다음 실행이 같은 채널을 그대로 다시 쓸 수 있다.
    }

protected:
    void sendPulse(Axis axis, int pulse_us) override {
        writeLong(duty_fd_[index(axis)], static_cast<long>(pulse_us) * 1000);
    }
    // duty_cycle 0 = 펄스 없음. 핀이 LOW 로 붙잡히고 서보는 힘을 놓는다.
    // enable 을 토글하지 않는 이유: 껐다 켜면 period 가 남아 있는지가 커널
    // 버전에 따라 다르고, 매번 다시 써야 하는지 알 수 없어진다.
    void sendRelease(Axis axis) override { writeLong(duty_fd_[index(axis)], 0); }

private:
    // 20ms(50Hz). servo.c 의 SERVO_PERIOD_US 와 같은 값이어야 비교가 공정하다.
    static constexpr int kPeriodUs = 20000;

    static size_t index(Axis axis) { return axis == Axis::Pan ? 0 : 1; }

    void openChannel(Axis axis, int ch) {
        const std::string dir = cfg_.syspwm_chip + "/pwm" + std::to_string(ch);

        // 이전 실행이 이미 열어 둔 채널이면 export 는 EBUSY 로 실패한다.
        // 그건 오류가 아니라 "이미 준비됨" 이므로 있는지 먼저 본다.
        if (::access(dir.c_str(), F_OK) != 0) {
            writeFile(cfg_.syspwm_chip + "/export", std::to_string(ch));
        }

        // export 는 디렉터리를 즉시 만들지만, 그 안의 파일을 gpio 그룹이 쓸 수
        // 있게 바꿔 주는 것은 udev 가 나중에 한다. 곧바로 열면 EACCES 가 난다.
        int fd = -1;
        for (int i = 0; i < 20 && fd < 0; ++i) {
            fd = ::open((dir + "/duty_cycle").c_str(), O_WRONLY | O_CLOEXEC);
            if (fd < 0) std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (fd < 0) {
            throw std::runtime_error(
                dir + "/duty_cycle 을 열지 못했습니다 (" + std::strerror(errno) +
                "). dtoverlay=pwm-2chan 이 올라갔는지(ls /sys/class/pwm),"
                " gpio 그룹에 속해 있는지 확인하세요.");
        }
        duty_fd_[index(axis)] = fd;

        // 순서가 중요하다. duty_cycle 은 "period 중 얼마" 라서 period 가 먼저
        // 있어야 받아들여진다. 게다가 이전 실행이 남긴 duty 가 새 period 보다
        // 크면 그 순간 duty > period 가 되어 period 쓰기가 거부된다.
        // 그래서 duty 를 0 으로 내린 뒤 period 를 쓴다.
        writeLong(fd, 0);
        writeFile(dir + "/period", std::to_string(kPeriodUs * 1000L));
        writeFile(dir + "/enable", "1");
    }

    // sysfs 는 write 한 번이 값 하나다. 이어 쓰지 않도록 오프셋 0 에 덮어쓴다.
    // 제어 루프마다(초당 5회 x 2채널) 불리므로 fd 는 열어 둔 채로 재사용한다.
    static void writeLong(int fd, long value) {
        if (fd < 0) return;
        char buf[32];
        const int n = std::snprintf(buf, sizeof(buf), "%ld", value);
        if (n <= 0) return;
        if (::pwrite(fd, buf, static_cast<size_t>(n), 0) != n) {
            LOG_W(TAG, "duty_cycle 쓰기 실패 (%s): %ld", std::strerror(errno), value);
        }
    }

    // 초기화용. 한 번만 쓰는 파일이라 그때그때 열고 닫는다. 실패하면 던진다 —
    // 여기서 실패하면 펄스가 아예 안 나가므로 조용히 넘길 수 없다.
    static void writeFile(const std::string& path, const std::string& value) {
        const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
        if (fd < 0) {
            throw std::runtime_error(path + " 를 열지 못했습니다 (" +
                                     std::strerror(errno) + ")");
        }
        const ssize_t n = ::write(fd, value.c_str(), value.size());
        const int err = errno;
        ::close(fd);
        if (n != static_cast<ssize_t>(value.size())) {
            throw std::runtime_error(path + " 에 \"" + value + "\" 를 쓰지 못했습니다 (" +
                                     std::strerror(err) + ")");
        }
    }

    int duty_fd_[2] = {-1, -1};
};

}  // namespace

std::unique_ptr<PanTilt> makeMotor(const Config& cfg) {
    const std::string& backend = cfg.motor.backend;

    if (backend == "dummy") {
        return std::make_unique<DummyPanTilt>(cfg.motor);
    }
    if (backend != "servo" && backend != "kservo" && backend != "syspwm") {
        throw std::runtime_error("알 수 없는 motor.backend: " + backend);
    }
    try {
        if (backend == "kservo") {
            return std::make_unique<KernelServoPanTilt>(cfg.motor);
        }
        if (backend == "syspwm") {
            return std::make_unique<SysPwmPanTilt>(cfg.motor);
        }
        return std::make_unique<ServoPanTilt>(cfg.motor);
    } catch (const std::exception& e) {
        LOG_E(TAG, "%s", e.what());
        LOG_W(TAG, "모터 없이 계속합니다 (dummy).");
        return std::make_unique<DummyPanTilt>(cfg.motor);
    }
}

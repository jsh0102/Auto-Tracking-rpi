#pragma once

#include <memory>

#include "config.hpp"

// 팬/틸트 모터. 각도 단위는 deg, 팬 +는 오른쪽, 틸트 +는 위쪽.
class PanTilt {
public:
    virtual ~PanTilt() = default;

    // 가동 범위(config 의 pan_min/max 등)를 넘으면 잘라낸다.
    virtual void moveTo(double pan, double tilt) = 0;
    virtual void position(double& pan, double& tilt) const = 0;

    // PWM 을 끊어 서보를 free 상태로 둔다. 지터와 발열이 줄어든다.
    virtual void release() = 0;

    // 하드웨어 비상정지가 걸려 있는가.
    //
    // 걸려 있으면 커널이 모든 명령을 거부한다. 그걸 모르고 계속 보내면
    // ① 실패 로그가 쏟아지고 ② moveTo 가 "갔다" 고 기록해 버려서, 해제하는
    // 순간 서보가 그 차이만큼 한 번에 튄다. 비상정지 직후의 급격한 움직임은
    // 가장 피해야 할 동작이다.
    //
    // 상태를 묻는 것이므로 sysfs 를 읽는다. /dev/button0 의 블로킹 read 는
    // "방금 눌렸다" 는 이벤트용이고, 그건 button_wait 이 쓴다.
    virtual bool emergencyStopped() const { return false; }

    void moveBy(double dpan, double dtilt) {
        double pan = 0;
        double tilt = 0;
        position(pan, tilt);
        moveTo(pan + dpan, tilt + dtilt);
    }
    virtual void home() = 0;
};

// cfg.motor.backend
//   servo   pigpiod 를 통해 제어 (유저스페이스에서 PWM 생성)
//   kservo  /dev/servo0 에 써서 제어 (커널 모듈이 hrtimer 로 PWM 생성)
//   syspwm  /sys/class/pwm 에 써서 제어 (칩의 하드웨어 PWM 회로가 생성)
//   dummy   아무것도 하지 않음
//
// servo/kservo/syspwm 인데 장치를 잡지 못하면 경고를 찍고 dummy 로 내려간다 —
// 모터가 없다고 송출까지 죽일 이유가 없다.
std::unique_ptr<PanTilt> makeMotor(const Config& cfg);

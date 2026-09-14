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

    void moveBy(double dpan, double dtilt) {
        double pan = 0;
        double tilt = 0;
        position(pan, tilt);
        moveTo(pan + dpan, tilt + dtilt);
    }
    virtual void home() = 0;
};

// cfg.motor.backend: servo | dummy.
// servo 인데 pigpiod 에 붙지 못하면 경고를 찍고 dummy 로 내려간다 —
// 모터가 없다고 송출까지 죽일 이유가 없다.
std::unique_ptr<PanTilt> makeMotor(const Config& cfg);

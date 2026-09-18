// 서보 캘리브레이션 / 안전 테스트 도구
//
// 각도가 아니라 펄스폭(µs)을 직접 다룬다. 각도 변환은 min_pulse_ms/max_pulse_ms
// 가 이 서보에 맞다는 전제가 필요한데, 지금은 그 값이 맞는지를 확인하려는 것이라
// 전제로 삼을 수 없다.
//
// 시작할 때 양쪽을 1500µs(정중앙)로 보낸다. 지금 서보가 어느 각도에 있는지 알 수
// 없으므로, 어느 쪽 끝에서든 안쪽으로 들어오는 중앙이 가장 안전하다.
//
//   빌드:  make tools
//   실행:  ./tools/servo_test

#include <pigpiod_if2.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include "config.hpp"

namespace {

// 서보의 물리적 한계. 이 밖으로 나가면 내부에서 기어가 끝까지 밀려 계속 힘을 쓴다.
constexpr int kAbsMinUs = 500;
constexpr int kAbsMaxUs = 2500;
constexpr int kCenterUs = 1500;

constexpr int kStepUs = 50;        // 약 5도
constexpr int kBigStepUs = 200;    // 대문자 키

int clampUs(int us) { return std::max(kAbsMinUs, std::min(kAbsMaxUs, us)); }

// 설정의 펄스폭 범위를 기준으로 한 추정 각도. 설정이 틀렸으면 이 값도 틀리다 —
// 어디까지나 참고용이고, 판단은 눈으로 본 실제 움직임으로 한다.
double toAngle(const MotorConfig& m, int us) {
    const double min_us = m.min_pulse_ms * 1000.0;
    const double max_us = m.max_pulse_ms * 1000.0;
    return (us - min_us) / (max_us - min_us) * 180.0 - 90.0;
}

const char* kHelp =
    "\n"
    "  a / d   팬  왼쪽 / 오른쪽      (A / D = 4배씩)\n"
    "  w / s   틸트 위 / 아래         (W / S = 4배씩)\n"
    "  c       양쪽 중앙(1500µs)으로\n"
    "  p       지금 팬 위치를 한계로 기록\n"
    "  t       지금 틸트 위치를 한계로 기록\n"
    "  h       이 도움말\n"
    "  q       PWM 끄고 종료 (설정값 출력)\n"
    "\n"
    "  ※ 한 글자 입력하고 Enter. 서보가 뻑뻑한 소리를 내면 즉시 반대로 돌릴 것\n";

}  // namespace

int main(int argc, char** argv) {
    const std::string cfg_path = (argc > 1) ? argv[1] : "config.json";
    Config cfg;
    try {
        cfg = Config::load(cfg_path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "설정 오류: %s\n", e.what());
        return 2;
    }
    const MotorConfig& m = cfg.motor;

    const int pi = pigpio_start(m.pigpio_host.c_str(), m.pigpio_port.c_str());
    if (pi < 0) {
        std::fprintf(stderr,
                     "pigpiod 에 연결하지 못했습니다 (%s:%s)\n"
                     "  sudo systemctl enable --now pigpiod\n",
                     m.pigpio_host.c_str(), m.pigpio_port.c_str());
        return 1;
    }
    std::printf("pigpiod 연결됨 — 팬 GPIO%d(물리 32번), 틸트 GPIO%d(물리 33번)\n",
                m.pan_pin, m.tilt_pin);

    int pan_us = kCenterUs;
    int tilt_us = kCenterUs;
    std::vector<int> pan_limits;
    std::vector<int> tilt_limits;

    auto apply = [&] {
        set_servo_pulsewidth(pi, static_cast<unsigned>(m.pan_pin),
                             static_cast<unsigned>(pan_us));
        set_servo_pulsewidth(pi, static_cast<unsigned>(m.tilt_pin),
                             static_cast<unsigned>(tilt_us));
    };

    std::printf("\n양쪽을 중앙(%dµs)으로 보냅니다...\n", kCenterUs);
    apply();
    std::printf("%s", kHelp);

    std::string line;
    while (true) {
        std::printf("\r  팬 %4dµs (%+6.1f도)   틸트 %4dµs (%+6.1f도)  > ",
                    pan_us, toAngle(m, pan_us), tilt_us, toAngle(m, tilt_us));
        std::fflush(stdout);

        if (!std::getline(std::cin, line)) break;
        if (line.empty()) continue;

        const char key = line[0];
        const int step = std::isupper(static_cast<unsigned char>(key)) ? kBigStepUs : kStepUs;
        const char k = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));

        if (k == 'q') break;
        else if (k == 'a') pan_us = clampUs(pan_us - step);
        else if (k == 'd') pan_us = clampUs(pan_us + step);
        else if (k == 'w') tilt_us = clampUs(tilt_us + step);
        else if (k == 's') tilt_us = clampUs(tilt_us - step);
        else if (k == 'c') { pan_us = kCenterUs; tilt_us = kCenterUs; }
        else if (k == 'p') {
            pan_limits.push_back(pan_us);
            std::printf("\n  팬 한계 기록: %dµs   (기록 %zu개)\n", pan_us, pan_limits.size());
            continue;
        } else if (k == 't') {
            tilt_limits.push_back(tilt_us);
            std::printf("\n  틸트 한계 기록: %dµs   (기록 %zu개)\n", tilt_us, tilt_limits.size());
            continue;
        } else {
            std::printf("%s", kHelp);
            continue;
        }
        apply();
    }

    // PWM 해제 — 펄스폭 0 이면 서보가 힘을 놓는다.
    set_servo_pulsewidth(pi, static_cast<unsigned>(m.pan_pin), 0);
    set_servo_pulsewidth(pi, static_cast<unsigned>(m.tilt_pin), 0);
    pigpio_stop(pi);

    std::printf("\n\nPWM 해제됨.\n");
    if (pan_limits.empty() && tilt_limits.empty()) return 0;

    std::printf("\n--- 기록된 한계 ---\n");
    auto report = [](const char* name, const std::vector<int>& v) {
        if (v.empty()) return;
        const auto mm = std::minmax_element(v.begin(), v.end());
        std::printf("  %s: %dµs ~ %dµs\n", name, *mm.first, *mm.second);
    };
    report("팬  ", pan_limits);
    report("틸트", tilt_limits);
    std::printf(
        "\n이 값들로 config.json 의 min_pulse_ms/max_pulse_ms 와 가동범위를\n"
        "정할 수 있습니다. 어떻게 환산할지는 같이 보죠.\n");
    return 0;
}

// button_poll — 유저스페이스 폴링으로 비상정지를 구현한다 (A 단계 기준선)
//
// 커널 모듈 없이 같은 기능을 만들면 무엇이 드는지 재기 위한 프로그램이다.
// 감지도, 정지 판단도, LED 도 전부 유저스페이스가 한다. 커널은 시키는 대로만 한다.
//
//   ./tools/button_poll --interval 10ms
//   ./tools/button_poll --interval 100us
//   ./tools/button_poll --busy            안 자고 계속 확인
//
// 폴링은 "지연"과 "CPU"를 맞바꾼다. 간격을 줄이면 반응이 빨라지지만 CPU 를 더 쓴다.
// 그 맞교환이 실제로 얼마인지를 재는 것이 이 프로그램의 목적이다.
//
// 진짜 지연 측정: 유저스페이스 혼자서는 "버튼이 실제로 눌린 시각" 을 알 수 없다.
// 알려면 그게 이미 인터럽트다. 그래서 커널 모듈이 인터럽트로 기록한 시각을
// /sys/class/servo/servo0/press 에서 읽어 기준시계로 쓴다.
//
//   지연 = 폴링이 알아챈 시각 − 커널이 인터럽트를 받은 시각
//
// 모듈이 없으면 이 측정만 건너뛰고 나머지(주기·CPU)는 그대로 동작한다.

#include <fcntl.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

constexpr int DEFAULT_BUTTON_GPIO = 17;
constexpr int DEFAULT_LED_GPIO = 27;
constexpr const char* DEFAULT_DEV = "/dev/servo0";
constexpr const char* DEFAULT_PRESS = "/sys/class/servo/servo0/press";
// 모듈이 GPIO17 을 쥐고 있으면 /sys/class/gpio 로는 못 연다(EBUSY).
// 그때는 드라이버가 내주는 이 파일을 폴링한다. 하는 일은 같다.
constexpr const char* DRIVER_LEVEL = "/sys/class/servo/servo0/level";
constexpr int SERVO_CENTER_US = 1500;

volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }

int64_t nowNs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

// ── sysfs GPIO ──────────────────────────────────────────────
// 유저스페이스에서 GPIO 를 다루는 옛 방식. 파일처럼 읽고 쓴다.
// pigpio 는 쓰지 않는다 — 비교 대상이므로 섞이면 안 된다.

bool writeFile(const std::string& path, const std::string& value) {
    int fd = ::open(path.c_str(), O_WRONLY);
    if (fd < 0) return false;
    const bool ok = ::write(fd, value.c_str(), value.size()) == (ssize_t)value.size();
    ::close(fd);
    return ok;
}

std::string gpioDir(int gpio) { return "/sys/class/gpio/gpio" + std::to_string(gpio); }

bool gpioExport(int gpio, const char* direction) {
    struct stat st;
    if (::stat(gpioDir(gpio).c_str(), &st) != 0) {
        if (!writeFile("/sys/class/gpio/export", std::to_string(gpio))) {
            std::fprintf(stderr, "GPIO%d export 실패: %s\n", gpio, std::strerror(errno));
            return false;
        }
        // udev 가 권한을 맞출 시간을 준다.
        for (int i = 0; i < 50; i++) {
            if (::access((gpioDir(gpio) + "/direction").c_str(), W_OK) == 0) break;
            usleep(20000);
        }
    }
    if (!writeFile(gpioDir(gpio) + "/direction", direction)) {
        std::fprintf(stderr, "GPIO%d direction 설정 실패: %s\n", gpio, std::strerror(errno));
        return false;
    }
    return true;
}

void gpioUnexport(int gpio) { writeFile("/sys/class/gpio/unexport", std::to_string(gpio)); }

// ── 서보 제어 ───────────────────────────────────────────────
// 커널 드라이버에 한 줄 쓴다. 펄스는 커널이 만들지만, "멈춰라" 는 판단은
// 여기(유저스페이스)에서 나와 시스템콜로 경계를 넘어 들어간다. 그 경계가 A 의 비용이다.
void servoSet(int fd, bool stopped) {
    if (fd < 0) return;
    char buf[64];
    int n;
    if (stopped) {
        n = std::snprintf(buf, sizeof(buf), "pan off\n");
        ::write(fd, buf, n);
        n = std::snprintf(buf, sizeof(buf), "tilt off\n");
        ::write(fd, buf, n);
    } else {
        n = std::snprintf(buf, sizeof(buf), "pan %dus\n", SERVO_CENTER_US);
        ::write(fd, buf, n);
        n = std::snprintf(buf, sizeof(buf), "tilt %dus\n", SERVO_CENTER_US);
        ::write(fd, buf, n);
    }
}

void usage(const char* argv0) {
    std::printf(
        "사용법: %s [옵션]\n"
        "\n"
        "      --interval <시간>  폴링 간격. 10ms / 500us / 100 (기본 10ms)\n"
        "      --busy             안 자고 계속 확인 (간격 무시)\n"
        "      --seconds <초>     측정 시간 (기본 30, 0 이면 Ctrl+C 까지)\n"
        "      --button <gpio>    버튼 핀 (기본 %d)\n"
        "      --led <gpio>       LED 핀 (기본 %d)\n"
        "      --dev <경로>       서보 장치 (기본 %s)\n"
        "      --press <경로>     커널 기준시계 (기본 %s)\n"
        "      --pin <경로>       버튼 값을 읽을 파일. 주면 sysfs GPIO export 를 건너뛴다\n"
        "                         (모듈이 핀을 쥐고 있을 때: %s)\n"
        "  -h, --help             이 도움말\n",
        argv0, DEFAULT_BUTTON_GPIO, DEFAULT_LED_GPIO, DEFAULT_DEV, DEFAULT_PRESS,
        DRIVER_LEVEL);
}

// "10ms", "500us", "1000" 을 마이크로초로.
long parseInterval(const char* s) {
    char* end = nullptr;
    const long v = std::strtol(s, &end, 10);
    if (end == s || v < 0) return -1;
    if (std::strcmp(end, "ms") == 0) return v * 1000;
    if (std::strcmp(end, "us") == 0 || *end == '\0') return v;
    if (std::strcmp(end, "s") == 0) return v * 1000000;
    return -1;
}

}  // namespace

int main(int argc, char** argv) {
    long interval_us = 10000;   // 10ms
    bool busy = false;
    long seconds = 30;
    int button_gpio = DEFAULT_BUTTON_GPIO;
    int led_gpio = DEFAULT_LED_GPIO;
    std::string dev = DEFAULT_DEV;
    std::string press_path = DEFAULT_PRESS;
    std::string pin_path;          // 비어 있으면 sysfs GPIO 를 export 해서 쓴다

    for (int i = 1; i < argc; i++) {
        const char* a = argv[i];
        auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : nullptr; };
        if (!std::strcmp(a, "--interval")) {
            const char* v = next();
            if (!v || (interval_us = parseInterval(v)) < 0) {
                std::fprintf(stderr, "간격을 읽을 수 없습니다\n");
                return 1;
            }
        } else if (!std::strcmp(a, "--busy")) {
            busy = true;
        } else if (!std::strcmp(a, "--seconds")) {
            const char* v = next();
            if (!v) return 1;
            seconds = std::strtol(v, nullptr, 10);
        } else if (!std::strcmp(a, "--button")) {
            const char* v = next();
            if (!v) return 1;
            button_gpio = std::atoi(v);
        } else if (!std::strcmp(a, "--led")) {
            const char* v = next();
            if (!v) return 1;
            led_gpio = std::atoi(v);
        } else if (!std::strcmp(a, "--dev")) {
            const char* v = next();
            if (!v) return 1;
            dev = v;
        } else if (!std::strcmp(a, "--press")) {
            const char* v = next();
            if (!v) return 1;
            press_path = v;
        } else if (!std::strcmp(a, "--pin")) {
            const char* v = next();
            if (!v) return 1;
            pin_path = v;
        } else if (!std::strcmp(a, "-h") || !std::strcmp(a, "--help")) {
            usage(argv[0]);
            return 0;
        } else {
            std::fprintf(stderr, "모르는 옵션: %s\n", a);
            usage(argv[0]);
            return 1;
        }
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // 모듈이 핀을 쥐고 있으면 sysfs GPIO 로는 못 연다. 그때는 --pin 으로
    // 드라이버가 내주는 파일을 쓴다. 그 경우 풀업도 모듈이 이미 켜 두었다.
    if (pin_path.empty()) {
        // sysfs 는 풀업을 못 켠다. 하드웨어 설정이라 raspi-gpio 에 맡긴다.
        // 풀업이 없으면 버튼을 안 눌러도 계속 LOW 로 읽힌다.
        char cmd[128];
        std::snprintf(cmd, sizeof(cmd), "raspi-gpio set %d ip pu 2>/dev/null", button_gpio);
        if (std::system(cmd) != 0) {
            std::fprintf(stderr, "경고: 풀업을 켜지 못했습니다. "
                                 "raspi-gpio set %d ip pu 를 직접 실행해 보세요\n", button_gpio);
        }
        if (!gpioExport(button_gpio, "in")) {
            std::fprintf(stderr, "  모듈이 GPIO%d 를 쥐고 있으면 이렇게 쓰세요:\n"
                                 "    --pin %s\n", button_gpio, DRIVER_LEVEL);
            return 1;
        }
        pin_path = gpioDir(button_gpio) + "/value";
    }
    if (!gpioExport(led_gpio, "out")) return 1;

    const int btn_fd = ::open(pin_path.c_str(), O_RDONLY);
    const int led_fd = ::open((gpioDir(led_gpio) + "/value").c_str(), O_WRONLY);
    if (btn_fd < 0 || led_fd < 0) {
        std::fprintf(stderr, "value 파일을 열지 못했습니다: %s\n", std::strerror(errno));
        return 1;
    }

    // 커널 기준시계. 없으면(모듈 미적재) 지연 측정만 건너뛴다.
    const int press_fd = ::open(press_path.c_str(), O_RDONLY);
    if (press_fd < 0)
        std::fprintf(stderr, "경고: %s 를 열지 못했습니다 (%s). "
                             "지연 측정 없이 진행합니다\n",
                     press_path.c_str(), std::strerror(errno));

    const int dev_fd = ::open(dev.c_str(), O_WRONLY);
    if (dev_fd < 0) {
        std::fprintf(stderr, "경고: %s 를 열지 못했습니다 (%s). LED 만 동작합니다\n",
                     dev.c_str(), std::strerror(errno));
    }

    // 읽기 1회 비용을 먼저 잰다. 폴링의 최소 단가다.
    double read_cost_us = 0.0;
    {
        char b[4];
        const int N = 10000;
        const int64_t t0 = nowNs();
        for (int i = 0; i < N; i++) ::pread(btn_fd, b, sizeof(b), 0);
        read_cost_us = (nowNs() - t0) / 1000.0 / N;
    }

    std::printf("버튼 %s, LED GPIO%d, 장치 %s\n", pin_path.c_str(), led_gpio, dev.c_str());
    if (busy) std::printf("모드: busy (안 자고 계속 확인)\n");
    else      std::printf("모드: sleep, 요청 간격 %ldus\n", interval_us);
    std::printf("읽기 1회 비용: %.2fus\n", read_cost_us);
    if (seconds > 0) std::printf("%ld초 측정합니다. 버튼을 눌러 보세요.\n\n", seconds);
    else             std::printf("Ctrl+C 까지 측정합니다. 버튼을 눌러 보세요.\n\n");

    // ── 폴링 루프 ───────────────────────────────────────────
    bool stopped = false;
    int prev = 1;             // 풀업이므로 평소 HIGH
    long presses = 0, loops = 0;

    // 커널 기준시계로 재는 진짜 지연
    unsigned long long kseq_last = 0;
    long measured = 0;
    int64_t lat_sum = 0, lat_min = INT64_MAX, lat_max = INT64_MIN;

    // "seq ns" 한 줄을 읽는다. 실패하면 false.
    auto readPress = [&](unsigned long long& seq, long long& ns) {
        if (press_fd < 0) return false;
        char b[64] = {0};
        if (::pread(press_fd, b, sizeof(b) - 1, 0) <= 0) return false;
        return std::sscanf(b, "%llu %lld", &seq, &ns) == 2;
    };

    if (press_fd >= 0) {
        long long ns = 0;
        readPress(kseq_last, ns);   // 시작 시점 번호를 기억해 둔다
    }

    ::write(led_fd, "0", 1);
    servoSet(dev_fd, false);

    rusage ru0;
    ::getrusage(RUSAGE_SELF, &ru0);
    const int64_t wall0 = nowNs();
    const int64_t deadline = seconds > 0 ? wall0 + seconds * 1000000000LL : 0;

    while (!g_stop) {
        char b[4] = {0};
        if (::pread(btn_fd, b, sizeof(b), 0) > 0) {
            const int level = (b[0] == '0') ? 0 : 1;

            // 떨어지는 엣지(HIGH→LOW)가 "방금 눌렸다" 는 뜻이다.
            if (prev == 1 && level == 0) {
                const int64_t t_user = nowNs();   // 알아챈 순간을 먼저 찍는다
                presses++;

                // 커널 번호가 올라올 때까지 잠깐 기다린다. 인터럽트 처리와
                // 폴링이 동시에 달리므로, 폴링이 먼저 LOW 를 볼 수 있다.
                // 시각은 이미 찍었으므로 확인이 늦어도 값은 안 밀린다.
                unsigned long long kseq = kseq_last;
                long long kns = 0;
                for (int t = 0; t < 50 && press_fd >= 0; t++) {
                    if (readPress(kseq, kns) && kseq != kseq_last) break;
                }

                if (press_fd >= 0 && kseq != kseq_last) {
                    // 커널이 새 누름으로 인정했다 → 진짜 지연
                    const int64_t lat = t_user - kns;
                    kseq_last = kseq;
                    measured++;
                    lat_sum += lat;
                    if (lat < lat_min) lat_min = lat;
                    if (lat > lat_max) lat_max = lat;

                    stopped = !stopped;
                    ::write(led_fd, stopped ? "1" : "0", 1);
                    servoSet(dev_fd, stopped);
                    std::printf("[%ld] 버튼 → %-4s  지연 %.1fus\n", presses,
                                stopped ? "정지" : "재개", lat / 1000.0);
                } else if (press_fd >= 0) {
                    // 번호가 그대로다 = 커널이 튐으로 버린 것.
                    // 폴링만으로는 이걸 구분할 수 없어 오작동했던 부분이다.
                    std::printf("[%ld] (튐으로 판정 — 커널이 인정하지 않음)\n",
                                presses);
                } else {
                    stopped = !stopped;
                    ::write(led_fd, stopped ? "1" : "0", 1);
                    servoSet(dev_fd, stopped);
                    std::printf("[%ld] 버튼 → %s\n", presses,
                                stopped ? "정지" : "재개");
                }
                std::fflush(stdout);
            }
            prev = level;
        }
        loops++;

        if (deadline && nowNs() >= deadline) break;

        if (!busy) {
            timespec ts{0, 0};
            ts.tv_sec = interval_us / 1000000;
            ts.tv_nsec = (interval_us % 1000000) * 1000;
            ::nanosleep(&ts, nullptr);
        }
    }

    const int64_t wall = nowNs() - wall0;
    rusage ru1;
    ::getrusage(RUSAGE_SELF, &ru1);

    auto usecOf = [](const rusage& a, const rusage& b) {
        return (b.ru_utime.tv_sec - a.ru_utime.tv_sec) * 1000000.0 +
               (b.ru_utime.tv_usec - a.ru_utime.tv_usec) +
               (b.ru_stime.tv_sec - a.ru_stime.tv_sec) * 1000000.0 +
               (b.ru_stime.tv_usec - a.ru_stime.tv_usec);
    };
    const double cpu_us = usecOf(ru0, ru1);
    const double wall_us = wall / 1000.0;
    const double actual_period_us = loops ? wall_us / loops : 0.0;

    // 정리 — 서보를 놓고 LED 를 끈다.
    servoSet(dev_fd, true);
    ::write(led_fd, "0", 1);
    if (dev_fd >= 0) ::close(dev_fd);
    if (press_fd >= 0) ::close(press_fd);
    ::close(btn_fd);
    ::close(led_fd);
    if (pin_path.rfind("/sys/class/gpio/", 0) == 0) gpioUnexport(button_gpio);
    gpioUnexport(led_gpio);

    std::printf("\n");
    std::printf("────────────────────────────────────────\n");
    std::printf(" 모드              %s\n", busy ? "busy" : "sleep");
    if (!busy) std::printf(" 요청 간격         %ldus\n", interval_us);
    std::printf(" 실제 평균 주기     %.1fus\n", actual_period_us);
    std::printf(" 루프 횟수         %ld\n", loops);
    std::printf(" 측정 시간         %.1f초\n", wall_us / 1000000.0);
    std::printf(" 이 프로세스 CPU   %.2f%% (1코어 기준), %.2f%% (4코어 기준)\n",
                wall_us > 0 ? cpu_us * 100.0 / wall_us : 0.0,
                wall_us > 0 ? cpu_us * 100.0 / wall_us / 4.0 : 0.0);
    std::printf(" 읽기 1회 비용     %.2fus\n", read_cost_us);
    std::printf(" 엣지 감지         %ld회\n", presses);
    std::printf("────────────────────────────────────────\n");
    if (measured > 0) {
        std::printf(" 진짜 지연 (커널 기준시계, %ld회)\n", measured);
        std::printf("   최소 %.1fus / 평균 %.1fus / 최대 %.1fus\n",
                    lat_min / 1000.0, (double)lat_sum / measured / 1000.0,
                    lat_max / 1000.0);
        std::printf(" 구조적 상한 (= 실제 주기) %.1fus\n", actual_period_us);
        std::printf("   눌리는 시점은 주기 안 아무 때나이므로 평균은 주기의 절반에\n");
        std::printf("   가까워야 한다.\n");
    } else {
        std::printf(" 진짜 지연         측정 못 함 (모듈이 적재됐는지 확인)\n");
        std::printf(" 최악 지연 = 실제 주기 = %.1fus\n", actual_period_us);
    }
    return 0;
}

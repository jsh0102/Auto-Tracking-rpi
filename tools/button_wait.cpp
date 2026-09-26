// button_wait — 커널 인터럽트를 기다려 비상정지를 처리한다 (B 단계)
//
// button_poll 과 하는 일은 같다. 다른 것은 "어떻게 아느냐" 하나뿐이다.
//
//   button_poll   while(1) { GPIO 읽기; 잠깐 쉬기 }     물어본다  → CPU 를 쓴다
//   button_wait   read(/dev/button0)                     기다린다  → CPU 0
//
// 커널이 인터럽트를 받은 시각(ktime)을 같이 돌려주므로, 유저가 깨어난 시각과
// 빼면 "커널이 안 순간부터 유저가 알기까지" 의 진짜 지연이 나온다.
// 폴링에서는 기준 시각이 없어 잴 수 없었던 값이다.
//
//   ./tools/button_wait --seconds 30

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

constexpr int DEFAULT_LED_GPIO = 27;
constexpr const char* DEFAULT_BUTTON_DEV = "/dev/button0";
constexpr const char* DEFAULT_SERVO_DEV = "/dev/servo0";
constexpr int SERVO_CENTER_US = 1500;

volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }

int64_t nowNs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);   // 커널의 ktime_get() 과 같은 시계
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

bool writeFile(const std::string& path, const std::string& value) {
    int fd = ::open(path.c_str(), O_WRONLY);
    if (fd < 0) return false;
    const bool ok = ::write(fd, value.c_str(), value.size()) == (ssize_t)value.size();
    ::close(fd);
    return ok;
}

std::string gpioDir(int gpio) { return "/sys/class/gpio/gpio" + std::to_string(gpio); }

bool gpioExportOut(int gpio) {
    struct stat st;
    if (::stat(gpioDir(gpio).c_str(), &st) != 0) {
        if (!writeFile("/sys/class/gpio/export", std::to_string(gpio))) return false;
        for (int i = 0; i < 50; i++) {
            if (::access((gpioDir(gpio) + "/direction").c_str(), W_OK) == 0) break;
            usleep(20000);
        }
    }
    return writeFile(gpioDir(gpio) + "/direction", "out");
}

void servoSet(int fd, bool stopped) {
    if (fd < 0) return;
    char buf[64];
    int n;
    if (stopped) {
        n = std::snprintf(buf, sizeof(buf), "pan off\n");  ::write(fd, buf, n);
        n = std::snprintf(buf, sizeof(buf), "tilt off\n"); ::write(fd, buf, n);
    } else {
        n = std::snprintf(buf, sizeof(buf), "pan %dus\n", SERVO_CENTER_US);  ::write(fd, buf, n);
        n = std::snprintf(buf, sizeof(buf), "tilt %dus\n", SERVO_CENTER_US); ::write(fd, buf, n);
    }
}

void usage(const char* argv0) {
    std::printf(
        "사용법: %s [옵션]\n"
        "\n"
        "      --seconds <초>   측정 시간 (기본 30, 0 이면 Ctrl+C 까지)\n"
        "      --led <gpio>     LED 핀 (기본 %d)\n"
        "      --button <경로>  버튼 장치 (기본 %s)\n"
        "      --servo <경로>   서보 장치 (기본 %s)\n"
        "  -h, --help           이 도움말\n",
        argv0, DEFAULT_LED_GPIO, DEFAULT_BUTTON_DEV, DEFAULT_SERVO_DEV);
}

}  // namespace

int main(int argc, char** argv) {
    long seconds = 30;
    int led_gpio = DEFAULT_LED_GPIO;
    std::string btn_dev = DEFAULT_BUTTON_DEV;
    std::string servo_dev = DEFAULT_SERVO_DEV;

    for (int i = 1; i < argc; i++) {
        const char* a = argv[i];
        auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : nullptr; };
        if (!std::strcmp(a, "--seconds")) {
            const char* v = next(); if (!v) return 1;
            seconds = std::strtol(v, nullptr, 10);
        } else if (!std::strcmp(a, "--led")) {
            const char* v = next(); if (!v) return 1;
            led_gpio = std::atoi(v);
        } else if (!std::strcmp(a, "--button")) {
            const char* v = next(); if (!v) return 1;
            btn_dev = v;
        } else if (!std::strcmp(a, "--servo")) {
            const char* v = next(); if (!v) return 1;
            servo_dev = v;
        } else if (!std::strcmp(a, "-h") || !std::strcmp(a, "--help")) {
            usage(argv[0]); return 0;
        } else {
            std::fprintf(stderr, "모르는 옵션: %s\n", a);
            usage(argv[0]); return 1;
        }
    }

    // SA_RESTART 를 빼야 read() 가 신호에 EINTR 로 깨어난다.
    // 기본 signal() 은 재시작시켜서 Ctrl+C 로 못 빠져나온다.
    struct sigaction sa {};
    sa.sa_handler = onSignal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGALRM, &sa, nullptr);

    const int btn_fd = ::open(btn_dev.c_str(), O_RDONLY);
    if (btn_fd < 0) {
        std::fprintf(stderr, "%s 를 열지 못했습니다 (%s)\n"
                             "  모듈이 적재됐는지: lsmod | grep servo\n"
                             "  udev 규칙이 최신인지 확인하세요\n",
                     btn_dev.c_str(), std::strerror(errno));
        return 1;
    }
    const int servo_fd = ::open(servo_dev.c_str(), O_WRONLY);
    if (servo_fd < 0)
        std::fprintf(stderr, "경고: %s 를 열지 못했습니다 (%s). LED 만 동작합니다\n",
                     servo_dev.c_str(), std::strerror(errno));

    int led_fd = -1;
    if (gpioExportOut(led_gpio))
        led_fd = ::open((gpioDir(led_gpio) + "/value").c_str(), O_WRONLY);
    if (led_fd < 0) std::fprintf(stderr, "경고: LED(GPIO%d) 를 쓰지 못합니다\n", led_gpio);

    std::printf("버튼 장치 %s, 서보 %s, LED GPIO%d\n",
                btn_dev.c_str(), servo_dev.c_str(), led_gpio);
    std::printf("read() 에서 잠들어 기다립니다 — 이 동안 CPU 를 쓰지 않습니다.\n");
    if (seconds > 0) std::printf("%ld초 측정합니다. 버튼을 눌러 보세요.\n\n", seconds);
    else             std::printf("Ctrl+C 까지 측정합니다. 버튼을 눌러 보세요.\n\n");

    bool stopped = false;
    long presses = 0;
    int64_t lat_sum = 0, lat_min = INT64_MAX, lat_max = 0;

    if (led_fd >= 0) ::write(led_fd, "0", 1);
    servoSet(servo_fd, false);

    rusage ru0; ::getrusage(RUSAGE_SELF, &ru0);
    const int64_t wall0 = nowNs();
    if (seconds > 0) alarm((unsigned)seconds);

    while (!g_stop) {
        char buf[128];
        const ssize_t n = ::read(btn_fd, buf, sizeof(buf) - 1);   // ← 여기서 잠든다
        if (n < 0) {
            if (errno == EINTR) break;          // 시간이 다 됐거나 Ctrl+C
            std::fprintf(stderr, "read 실패: %s\n", std::strerror(errno));
            break;
        }
        const int64_t user_ns = nowNs();        // 깨어난 순간을 곧바로 찍는다
        buf[n] = '\0';

        // "press 3 ktime=123456789012"
        unsigned long long seq = 0;
        long long kernel_ns = 0;
        if (std::sscanf(buf, "press %llu ktime=%lld", &seq, &kernel_ns) != 2) {
            std::fprintf(stderr, "형식을 읽지 못했습니다: %s", buf);
            continue;
        }

        const int64_t latency = user_ns - kernel_ns;
        presses++;
        lat_sum += latency;
        if (latency < lat_min) lat_min = latency;
        if (latency > lat_max) lat_max = latency;

        stopped = !stopped;
        if (led_fd >= 0) ::write(led_fd, stopped ? "1" : "0", 1);
        servoSet(servo_fd, stopped);

        std::printf("[%llu] %s   커널→유저 %.1fus\n", seq,
                    stopped ? "정지" : "재개", latency / 1000.0);
        std::fflush(stdout);
    }
    alarm(0);

    const double wall_us = (nowNs() - wall0) / 1000.0;
    rusage ru1; ::getrusage(RUSAGE_SELF, &ru1);
    const double cpu_us =
        (ru1.ru_utime.tv_sec - ru0.ru_utime.tv_sec) * 1000000.0 +
        (ru1.ru_utime.tv_usec - ru0.ru_utime.tv_usec) +
        (ru1.ru_stime.tv_sec - ru0.ru_stime.tv_sec) * 1000000.0 +
        (ru1.ru_stime.tv_usec - ru0.ru_stime.tv_usec);

    servoSet(servo_fd, true);
    if (led_fd >= 0) { ::write(led_fd, "0", 1); ::close(led_fd); }
    if (servo_fd >= 0) ::close(servo_fd);
    ::close(btn_fd);
    writeFile("/sys/class/gpio/unexport", std::to_string(led_gpio));

    std::printf("\n");
    std::printf("────────────────────────────────────────\n");
    std::printf(" 방식              커널 IRQ + 유저 정지 (B)\n");
    std::printf(" 측정 시간         %.1f초\n", wall_us / 1000000.0);
    std::printf(" 버튼 눌림         %ld회\n", presses);
    if (presses) {
        std::printf(" 커널→유저 지연     최소 %.1fus / 평균 %.1fus / 최대 %.1fus\n",
                    lat_min / 1000.0, (double)lat_sum / presses / 1000.0,
                    lat_max / 1000.0);
    }
    std::printf(" 이 프로세스 CPU   %.3f%% (1코어 기준)\n",
                wall_us > 0 ? cpu_us * 100.0 / wall_us : 0.0);
    std::printf("────────────────────────────────────────\n");
    std::printf(" 폴링과 달리 기다리는 동안 CPU 를 쓰지 않는다.\n");
    std::printf(" 남은 지연은 '커널이 유저를 깨우고 스케줄러가 CPU 를 줄 때까지' 다.\n");
    return 0;
}

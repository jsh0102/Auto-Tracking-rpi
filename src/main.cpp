// camtracker — 라즈베리파이 RTSP 송출 + 사람 추적 팬/틸트 카메라
//
//   ./camtracker [-c config.json] [--no-track] [--no-stream]
//                [--log-level DEBUG] [--dump-config]

#include <csignal>
#include <cstdio>
#include <cstring>
#include <exception>
#include <string>

#include "app.hpp"
#include "config.hpp"
#include "detector.hpp"
#include "log.hpp"

namespace {

// 시그널 핸들러에서 안전하게 건드릴 수 있는 유일한 타입.
volatile std::sig_atomic_t g_stop = 0;

void onSignal(int /*sig*/) { g_stop = 1; }

void usage(const char* argv0) {
    std::printf(
        "사용법: %s [옵션]\n"
        "\n"
        "  -c, --config <경로>    설정 파일 (기본: config.json)\n"
        "      --dump-config      적용될 설정을 출력하고 종료\n"
        "      --check-model      검출 모델을 읽고 추론 1회를 돌려 본 뒤 종료\n"
        "      --no-track         모터 제어 없이 송출만\n"
        "      --no-stream        RTSP 송출 없이 추적만\n"
        "      --log-level <레벨> DEBUG | INFO | WARNING | ERROR\n"
        "  -h, --help             이 도움말\n",
        argv0);
}

}  // namespace

int main(int argc, char** argv) {
    std::string config_path = "config.json";
    std::string log_level_override;
    bool dump_config = false;
    bool check_model = false;
    bool no_track = false;
    bool no_stream = false;

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        auto needsValue = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s 에는 값이 필요합니다\n", name);
                std::exit(2);
            }
            return argv[++i];
        };

        if (std::strcmp(a, "-c") == 0 || std::strcmp(a, "--config") == 0) {
            config_path = needsValue(a);
        } else if (std::strcmp(a, "--log-level") == 0) {
            log_level_override = needsValue(a);
        } else if (std::strcmp(a, "--dump-config") == 0) {
            dump_config = true;
        } else if (std::strcmp(a, "--check-model") == 0) {
            check_model = true;
        } else if (std::strcmp(a, "--no-track") == 0) {
            no_track = true;
        } else if (std::strcmp(a, "--no-stream") == 0) {
            no_stream = true;
        } else if (std::strcmp(a, "-h") == 0 || std::strcmp(a, "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            std::fprintf(stderr, "알 수 없는 옵션: %s\n", a);
            usage(argv[0]);
            return 2;
        }
    }

    Config cfg;
    try {
        cfg = Config::load(config_path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "설정 오류: %s\n", e.what());
        return 2;
    }

    if (no_track) cfg.track.enabled = false;
    if (no_stream) cfg.stream.enabled = false;
    if (!log_level_override.empty()) cfg.debug.log_level = log_level_override;

    if (dump_config) {
        std::printf("%s\n", cfg.dump().c_str());
        return 0;
    }

    if (check_model) {
        std::string error;
        if (checkModel(cfg, error)) {
            std::printf("모델 OK — %s\n", cfg.detect.model_dir.c_str());
            return 0;
        }
        std::fprintf(stderr, "모델 이상: %s\n", error.c_str());
        return 1;
    }

    logSetLevel(cfg.debug.log_level.c_str());

    // 자식(ffmpeg)이 먼저 죽었을 때 write 로 프로세스가 통째로 죽지 않게 한다.
    std::signal(SIGPIPE, SIG_IGN);
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    try {
        App app(cfg);
        return app.run(g_stop);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "치명적 오류: %s\n", e.what());
        return 1;
    }
}

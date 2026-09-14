#include "log.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <mutex>

namespace {

LogLevel g_level = LogLevel::Info;
std::mutex g_mutex;  // 여러 스레드가 한 줄을 쪼개 쓰지 않도록

const char* levelName(LogLevel level) {
    switch (level) {
        case LogLevel::Debug:   return "DEBUG";
        case LogLevel::Info:    return "INFO";
        case LogLevel::Warning: return "WARNING";
        case LogLevel::Error:   return "ERROR";
    }
    return "?";
}

}  // namespace

void logSetLevel(const char* name) {
    if (name == nullptr) return;
    if (strcasecmp(name, "DEBUG") == 0)        g_level = LogLevel::Debug;
    else if (strcasecmp(name, "INFO") == 0)    g_level = LogLevel::Info;
    else if (strcasecmp(name, "WARNING") == 0) g_level = LogLevel::Warning;
    else if (strcasecmp(name, "ERROR") == 0)   g_level = LogLevel::Error;
}

bool logEnabled(LogLevel level) { return level >= g_level; }

void logMessage(LogLevel level, const char* tag, const char* fmt, ...) {
    if (!logEnabled(level)) return;

    char stamp[16] = "--:--:--";
    std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
    if (localtime_r(&now, &tm_buf) != nullptr) {
        std::strftime(stamp, sizeof(stamp), "%H:%M:%S", &tm_buf);
    }

    char body[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(body, sizeof(body), fmt, args);
    va_end(args);

    std::lock_guard<std::mutex> lock(g_mutex);
    // 로그는 stderr 로. stdout 은 --dump-config 같은 "결과" 전용으로 비워둔다.
    std::fprintf(stderr, "%s %-7s %s: %s\n", stamp, levelName(level), tag, body);
    std::fflush(stderr);
}

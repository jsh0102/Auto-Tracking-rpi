#pragma once

// printf 형식의 아주 작은 로거. 출력 형식은 파이썬 판과 맞췄다:
//   15:23:03 INFO    camera: 시작
enum class LogLevel { Debug = 0, Info, Warning, Error };

void logSetLevel(const char* name);   // "DEBUG" | "INFO" | "WARNING" | "ERROR"
bool logEnabled(LogLevel level);

// __attribute__((format)) 로 g++ 이 포맷 문자열과 인자 개수/타입을 검사하게 한다.
void logMessage(LogLevel level, const char* tag, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));

#define LOG_D(tag, ...) logMessage(LogLevel::Debug, tag, __VA_ARGS__)
#define LOG_I(tag, ...) logMessage(LogLevel::Info, tag, __VA_ARGS__)
#define LOG_W(tag, ...) logMessage(LogLevel::Warning, tag, __VA_ARGS__)
#define LOG_E(tag, ...) logMessage(LogLevel::Error, tag, __VA_ARGS__)

#pragma once

#include <memory>

#include "camera.hpp"
#include "config.hpp"

// libcamera 를 직접 열어 프레임을 받는 소스 (camera.backend = "direct").
//
// camera.cpp 의 LibcameraSource 와 달리 외부 프로세스를 쓰지 않는다.
// 압축·압축해제가 없으므로 검출용 프레임을 얻는 데 드는 CPU 가 거의 0 이다.
//
// 실패하면 std::runtime_error 를 던진다.
std::unique_ptr<FrameSource> makeDirectSource(const Config& cfg);

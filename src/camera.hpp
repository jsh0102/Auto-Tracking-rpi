#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "config.hpp"

// 검출 루프에 BGR 프레임을 공급하는 소스.
//
// [1단계 구현 메모]
// 카메라를 직접 열지 않고 libcamera-vid 에게 맡긴 뒤 결과를 파이프로 받는다.
// 파이프라인은 파이썬 판과 동일:
//
//   libcamera-vid --(H.264, stdout)--> ffmpeg --+-- copy -------> RTSP push
//                                               '-- scale+bgr24 -> 우리 stdin
//
// 그래서 검출용 프레임은 "압축했다가 다시 푼" 것이다. 2단계에서 libcamera
// C++ API 로 바꾸면 이 왕복이 사라진다.
class FrameSource {
public:
    virtual ~FrameSource() = default;

    virtual bool start() = 0;

    // 가장 최신 프레임 1장을 out 으로 옮긴다(복사 아님).
    // timeout_ms 안에 새 프레임이 없으면 false.
    virtual bool read(cv::Mat& out, int timeout_ms) = 0;

    virtual void stop() = 0;

    // 캡처 파이프라인이 아직 살아 있는지. 죽었으면 앱이 루프를 끝낸다.
    virtual bool alive() = 0;

    // 송출 영상 위에 그릴 박스를 알려 준다. 좌표는 read() 가 준 프레임 기준이며,
    // 송출 해상도와 다르면 구현이 알아서 비례 변환한다.
    // 송출을 직접 하지 않는 백엔드는 무시한다(기본 동작).
    virtual void setOverlay(const std::vector<cv::Rect>& /*boxes*/,
                            const std::string& /*label*/) {}
};

// cfg.camera.backend 에 따라 소스를 만든다. 모르는 backend 면 std::runtime_error.
std::unique_ptr<FrameSource> makeFrameSource(const Config& cfg);

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "config.hpp"

// 검출 루프에 BGR 프레임을 공급하는 소스.
//
// 구현은 libcamera 를 직접 열어 두 스트림을 동시에 받는다:
//   main  1280x720 YUV420 → 박스 오버레이 → ffmpeg 인코딩 → RTSP
//   lores  320x240 NV12   → BGR 변환      → 검출
//
// 압축·압축해제 왕복이 없다. 외부 프로세스는 인코딩을 맡은 ffmpeg 하나뿐이다.
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

std::unique_ptr<FrameSource> makeFrameSource(const Config& cfg);

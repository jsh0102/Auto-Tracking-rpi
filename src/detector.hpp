#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "config.hpp"

// 픽셀 좌표계 바운딩 박스 하나.
struct Detection {
    cv::Rect box;
    float score = 0.0F;

    cv::Point2f center() const {
        return {box.x + box.width * 0.5F, box.y + box.height * 0.5F};
    }
    int area() const { return box.width * box.height; }
};

class Detector {
public:
    virtual ~Detector() = default;
    virtual std::vector<Detection> detect(const cv::Mat& frame) = 0;
};

// cfg.detect.backend 에 따라 만든다: ssd | none.
// ssd 인데 모델 파일이 없으면 경고를 찍고 none 으로 내려간다 —
// 모델이 없다고 프로그램 전체가 못 뜨면 송출까지 같이 죽는다.
std::unique_ptr<Detector> makeDetector(const Config& cfg);

// 설정된 모델을 실제로 읽고 추론 1회를 돌려 본다. 성공하면 true.
// 실패하면 error 에 이유를 채운다. makeDetector 와 달리 조용히 넘어가지 않는다 —
// `--check-model` 이 쓰는 엄격한 경로다.
bool checkModel(const Config& cfg, std::string& error);

// 추적할 대상 1명 선택.
// 직전 타겟이 있으면 중심이 가장 가까운 검출을 이어서 따라가고(sticky),
// 없으면 가장 큰 박스 = 가장 가까이 있는 사람을 고른다.
// 아무것도 못 고르면 has_out 을 false 로 둔다.
bool selectTarget(const std::vector<Detection>& dets, const Detection* previous,
                  cv::Size frame_size, Detection& out);

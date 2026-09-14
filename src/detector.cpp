#include "detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <sys/stat.h>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "log.hpp"

namespace {

constexpr const char* TAG = "detector";

bool fileExists(const std::string& path) {
    struct stat st {};
    return ::stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

class NullDetector : public Detector {
public:
    std::vector<Detection> detect(const cv::Mat&) override { return {}; }
};

// MobileNet-SSD (Caffe, VOC 20 클래스). person = 15.
class MobileNetSSDDetector : public Detector {
public:
    static constexpr int kPersonClass = 15;
    static constexpr int kInput = 300;

    MobileNetSSDDetector(const std::string& proto, const std::string& weights,
                         float confidence)
        : confidence_(confidence) {
        net_ = cv::dnn::readNetFromCaffe(proto, weights);
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }

    std::vector<Detection> detect(const cv::Mat& frame) override {
        const cv::Mat blob = cv::dnn::blobFromImage(
            frame, 1 / 127.5, cv::Size(kInput, kInput),
            cv::Scalar(127.5, 127.5, 127.5), /*swapRB=*/false);
        net_.setInput(blob);
        const cv::Mat raw = net_.forward();

        // raw 는 1x1xNx7. 각 행: [batch, classId, score, x1, y1, x2, y2] (0~1 정규화)
        const int rows = raw.size[2];
        const auto* p = reinterpret_cast<const float*>(raw.data);

        std::vector<Detection> out;
        for (int i = 0; i < rows; ++i) {
            const float* row = p + i * 7;
            const float score = row[2];
            if (score < confidence_) continue;
            if (static_cast<int>(row[1]) != kPersonClass) continue;

            const int x1 = std::max(0, static_cast<int>(row[3] * frame.cols));
            const int y1 = std::max(0, static_cast<int>(row[4] * frame.rows));
            const int x2 = std::min(frame.cols - 1, static_cast<int>(row[5] * frame.cols));
            const int y2 = std::min(frame.rows - 1, static_cast<int>(row[6] * frame.rows));
            if (x2 > x1 && y2 > y1) {
                out.push_back(Detection{cv::Rect(x1, y1, x2 - x1, y2 - y1), score});
            }
        }
        return out;
    }

private:
    cv::dnn::Net net_;
    float confidence_;
};

}  // namespace

std::unique_ptr<Detector> makeDetector(const Config& cfg) {
    const std::string& backend = cfg.detect.backend;
    if (backend == "none") return std::make_unique<NullDetector>();
    if (backend != "ssd") {
        throw std::runtime_error("알 수 없는 detect.backend: " + backend);
    }

    const std::string proto = cfg.detect.model_dir + "/MobileNetSSD_deploy.prototxt";
    const std::string weights = cfg.detect.model_dir + "/MobileNetSSD_deploy.caffemodel";
    if (!fileExists(proto) || !fileExists(weights)) {
        LOG_W(TAG,
              "모델 파일이 없어 검출을 끕니다 (%s / %s). "
              "scripts/download_model.sh 를 실행하세요.",
              proto.c_str(), weights.c_str());
        return std::make_unique<NullDetector>();
    }
    try {
        LOG_I(TAG, "MobileNet-SSD 로드: %s", weights.c_str());
        return std::make_unique<MobileNetSSDDetector>(
            proto, weights, static_cast<float>(cfg.detect.confidence));
    } catch (const cv::Exception& e) {
        LOG_E(TAG, "모델 로드 실패, 검출을 끕니다: %s", e.what());
        return std::make_unique<NullDetector>();
    }
}

bool checkModel(const Config& cfg, std::string& error) {
    if (cfg.detect.backend != "ssd") {
        error = "detect.backend 가 'ssd' 가 아닙니다 (" + cfg.detect.backend + ")";
        return false;
    }
    const std::string proto = cfg.detect.model_dir + "/MobileNetSSD_deploy.prototxt";
    const std::string weights = cfg.detect.model_dir + "/MobileNetSSD_deploy.caffemodel";
    for (const std::string& f : {proto, weights}) {
        if (!fileExists(f)) {
            error = "파일이 없습니다: " + f;
            return false;
        }
    }
    try {
        MobileNetSSDDetector det(proto, weights, static_cast<float>(cfg.detect.confidence));
        // 읽기만 해서는 부족하다. prototxt 와 caffemodel 의 짝이 어긋난 경우는
        // 추론을 한 번 돌려 봐야 드러난다.
        cv::Mat probe(cfg.detect.height, cfg.detect.width, CV_8UC3, cv::Scalar(0, 0, 0));
        det.detect(probe);
    } catch (const cv::Exception& e) {
        error = e.what();
        return false;
    }
    return true;
}

bool selectTarget(const std::vector<Detection>& dets, const Detection* previous,
                  cv::Size frame_size, Detection& out) {
    if (dets.empty()) return false;

    auto biggest = [&dets]() {
        return *std::max_element(dets.begin(), dets.end(),
                                 [](const Detection& a, const Detection& b) {
                                     return a.area() < b.area();
                                 });
    };

    if (previous == nullptr) {
        out = biggest();
        return true;
    }

    const cv::Point2f prev = previous->center();
    const Detection& nearest = *std::min_element(
        dets.begin(), dets.end(), [&prev](const Detection& a, const Detection& b) {
            return cv::norm(a.center() - prev) < cv::norm(b.center() - prev);
        });

    // 직전 위치에서 너무 멀면 다른 사람으로 보고 가장 큰 박스로 재선택한다.
    const double diag = std::hypot(frame_size.width, frame_size.height);
    out = (cv::norm(nearest.center() - prev) > 0.35 * diag) ? biggest() : nearest;
    return true;
}

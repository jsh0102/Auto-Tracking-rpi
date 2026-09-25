#pragma once

#include <string>

// 실행 설정. 기본값은 여기 적힌 값이고, config.json 에 적힌 키만 덮어쓴다.
// 모르는 키가 있으면 실행을 거부한다 — 오타 난 설정을 조용히 무시하지 않기 위해서다.

struct CameraConfig {
    int width = 1280;                   // 송출 해상도
    int height = 720;
    int fps = 30;
    int bitrate = 3000000;              // H.264 비트레이트 (bps)
    int rotation = 0;                   // 0 | 180
    bool hflip = false;
    bool vflip = false;
};

struct StreamConfig {
    bool enabled = true;
    std::string rtsp_url = "rtsp://127.0.0.1:8554/cam";
    std::string rtsp_transport = "tcp";  // tcp | udp
    std::string ffmpeg_loglevel = "warning";
};

struct DetectConfig {
    int width = 320;                    // 검출용 저해상도 프레임
    int height = 240;
    int fps = 10;
    std::string backend = "ssd";        // ssd | none
    double confidence = 0.5;
    std::string model_dir = "models";
    int interval = 1;                   // N 프레임마다 1회 처리

    // 검출 보간: 무거운 모델은 가끔만 돌리고 사이는 가벼운 추적으로 메운다.
    // 모델 추론이 약 220ms 인 반면 추적은 수 ms 라 처리량이 크게 오른다.
    std::string tracker = "kcf";         // kcf | csrt | none(보간 끔)
    int redetect_interval = 5;           // N 프레임마다 실제 검출로 보정
};

struct MotorConfig {
    std::string backend = "servo";      // servo(pigpio) | kservo(커널 드라이버) | dummy
    int pan_pin = 12;                   // BCM 번호
    int tilt_pin = 13;
    // MG90(S) 기본값. 개체차가 크므로 scripts/calibrate_servo.py 로 실측 권장.
    double min_pulse_ms = 0.5;
    double max_pulse_ms = 2.4;
    double pan_min = -80.0;             // 기구적 가동 범위 (deg)
    double pan_max = 80.0;
    double tilt_min = -35.0;
    double tilt_max = 50.0;
    double pan_home = 0.0;
    double tilt_home = 0.0;
    // 서보가 의도와 반대로 도는 경우에만 true (배선/기구 방향 보정용).
    // 화면 y축 부호는 tracker 에서 이미 처리하므로 여기서 건드리지 않는다.
    bool pan_invert = false;
    bool tilt_invert = false;
    bool idle_detach = true;            // 정지 시 PWM 해제 (지터/발열 감소)
    std::string pigpio_host = "localhost";   // backend=servo 일 때만 쓴다
    std::string pigpio_port = "8888";
    // backend=kservo 일 때 쓸 장치 파일. kernel/servo/servo.ko 가 만든다.
    std::string kservo_path = "/dev/servo0";
};

struct PIDConfig {
    double kp = 18.0;                   // 정규화 오차(-1..1) -> 각도 변화량(deg)
    double ki = 0.0;
    double kd = 3.0;
    double max_step = 6.0;              // 1틱당 최대 이동 각도
    double integral_limit = 5.0;
};

struct TrackConfig {
    bool enabled = true;
    double deadzone = 0.08;             // 화면 중심 기준 무시 영역 (정규화)
    double lost_timeout = 3.0;          // 타겟 소실 후 홈 복귀까지 대기(초)
    bool recenter_on_lost = true;

    // 지연 보상 — 이미 내렸지만 아직 영상에 안 나타난 명령을 오차에서 뺀다.
    //
    // 검출·추적 결과는 과거 장면을 본 값이라, 방금 보낸 명령이 반영돼 있지 않다.
    // 보정하지 않으면 같은 오차로 여러 번 명령해 카메라가 크게 오버슈트한다.
    //
    // deg_to_err: 1도 명령이 만드는 정규화 오차 변화량. 실측으로 구한 값이다.
    //   로그에서 각도 변화 대비 박스 이동을 재면 6.4 픽셀/도 (수평 화각 약 50도).
    //   pan  = 6.4 / 160(반폭)  = 0.040
    //   tilt = 6.4 / 120(반높이) = 0.053
    bool compensate_latency = true;
    double latency_ms = 300.0;      // 명령이 영상에 나타나기까지 걸리는 시간
    double pan_deg_to_err = 0.040;
    double tilt_deg_to_err = 0.053;

    PIDConfig pan_pid;
    PIDConfig tilt_pid{14.0, 0.0, 2.5, 6.0, 5.0};
};

struct DebugConfig {
    std::string log_level = "INFO";     // DEBUG | INFO | WARNING | ERROR
    std::string snapshot_path;          // 비우면 비활성
    double snapshot_interval = 1.0;     // 초
};

struct Config {
    CameraConfig camera;
    StreamConfig stream;
    DetectConfig detect;
    MotorConfig motor;
    TrackConfig track;
    DebugConfig debug;

    // 파일이 없으면 기본값을 그대로 돌려준다.
    // JSON 문법 오류나 모르는 키가 있으면 std::runtime_error 를 던진다 —
    // 오타 난 설정을 조용히 무시하면 "왜 안 먹지" 로 한참을 잃는다.
    static Config load(const std::string& path);

    std::string dump() const;           // 적용된 전체 설정을 JSON 으로
};

# camtracker

라즈베리파이 4 + IMX219(Pi Camera v2) + MG90 서보 2개로
**RTSP 실시간 송출**과 **사람 추적 팬/틸트**를 수행한다. C++17.

## 현재 상태

| 기능 | 상태 |
|---|---|
| RTSP 송출 | 동작 확인됨 (1280x720 H.264 30fps, 바운딩 박스 포함) |
| 사람 검출 | 동작 확인됨 (MobileNet-SSD, 약 4~6 fps) |
| 추적 (PID) | 코드 작성됨. 가짜 입력으로 부호만 확인, 실제 서보 미검증 |
| 서보 제어 | 코드 작성됨. **하드웨어 미연결, 한 번도 실행된 적 없음** |

## 동작 구조

```
  ISP ──── 하드웨어로 두 스트림 동시 출력 ────┐
   │                                          │
   ▼ 1280x720 YUV420           320x240 NV12   ▼
 [ 박스 오버레이 ]                    [ NV12 -> BGR ]
   │                                          │
   │                              MobileNet-SSD 검출
   │                                          │
   │                    박스 좌표 ◄───────────┤
   ▼                                          ▼
 ffmpeg (h264_v4l2m2m)              PID -> pigpiod -> MG90 x2
   │  인코딩만. 디코딩 없음.
   ▼
 MediaMTX :8554 ──→ 시청자
```

libcamera C++ API 로 카메라를 직접 연다. ISP 가 두 해상도를 하드웨어로 동시에
출력하므로 소프트웨어 축소가 없고, 검출용 프레임은 압축을 거치지 않는다.
외부 프로세스는 인코딩을 맡은 ffmpeg 하나뿐이다.

검출 좌표는 송출 프레임의 Y(밝기) 평면에 그려진다. 색 평면을 건드리지 않으므로
변환 비용이 붙지 않는다 — 박스는 흰색으로 보인다.

## 빠른 시작

```bash
./scripts/install.sh          # libcamera-apps, ffmpeg, pigpiod, MediaMTX
./scripts/download_model.sh   # MobileNet-SSD 가중치 (~23MB)
make                          # 빌드
make run                      # MediaMTX 까지 같이 실행
```

실행 인자는 `ARGS` 로 넘긴다:

```bash
make run ARGS="--no-track"    # 모터 없이 송출만
```

시청:

```bash
ffplay -fflags nobuffer -flags low_delay -rtsp_transport tcp rtsp://<파이IP>:8554/cam
```

VLC, 브라우저(`http://<파이IP>:8889/cam`, WebRTC)로도 볼 수 있다.

## 빌드

```bash
make          # 빌드 (바뀐 파일만 재컴파일)
make clean    # 빌드 결과물 삭제
```

필요한 개발 패키지: `libopencv-dev`, `libpigpio-dev`(헤더), `g++`, `make`.
`third_party/nlohmann/json.hpp` 는 저장소에 포함되어 있다.

## 하드웨어 연결

| 서보 | BCM 핀 | 비고 |
|------|--------|------|
| 팬(pan)   | GPIO12 | 하드웨어 PWM 채널 0 |
| 틸트(tilt)| GPIO13 | 하드웨어 PWM 채널 1 |

- 서보 전원(빨강)은 **파이 5V 핀이 아니라 별도 5V 공급**에 연결할 것.
  MG90S 는 스톨 시 700mA 이상을 먹어 파이가 리셋된다. GND 는 파이와 공통으로 묶는다.
- `pigpiod` 데몬이 떠 있어야 한다: `sudo systemctl enable --now pigpiod`.
  없으면 경고를 찍고 모터만 비활성화된 채로 계속 동작한다(송출은 살아 있다).

## 설정 (`config.json`)

전체 기본값은 `./build/camtracker --dump-config` 로 확인.

| 키 | 의미 |
|----|------|
| `camera.width/height/fps/bitrate` | 송출 해상도·비트레이트 |
| `stream.rtsp_url` | 푸시할 RTSP 주소 |
| `detect.backend` | `ssd` \| `none` |
| `detect.width/height/fps` | 검출 파이프라인 해상도·프레임레이트 |
| `detect.interval` | N 프레임마다 1회 검출 (CPU 절약) |
| `motor.pan_min/max`, `tilt_min/max` | 기구적 가동 범위(deg) |
| `motor.pan_invert`, `tilt_invert` | 서보가 반대로 돌 때만 `true` |
| `track.deadzone` | 화면 중앙 이 비율 안에서는 움직이지 않음 |
| `track.pan_pid.kp/ki/kd`, `max_step` | 추적 응답성. 떨리면 `kp`↓ `kd`↑ |
| `debug.snapshot_path` | 비우면 꺼짐. 경로를 주면 오버레이 JPEG 를 주기적으로 저장 |

설정 파일에 없는 키를 쓰면 실행을 거부한다(오타를 조용히 무시하지 않는다).

## 모델 확인

```bash
./build/camtracker --check-model
```

모델을 실제로 읽고 추론 1회를 돌려 본다. `prototxt` 와 `caffemodel` 의 짝이
어긋난 경우는 파일을 읽는 것만으로는 드러나지 않아서, 추론까지 해봐야 한다.

## CLI

```
./build/camtracker [-c config.json] [--no-track] [--no-stream]
                   [--log-level DEBUG] [--dump-config]
```

## 부팅 시 자동 실행

```bash
./systemd/install-services.sh
journalctl -fu camtracker
```

## 코드 구성

| 파일 | 역할 |
|------|------|
| `src/camera.cpp` | libcamera 직접 제어, 듀얼 스트림, 박스 오버레이, 인코더 파이프 |
| `src/detector.cpp` | `Detector` 인터페이스, MobileNet-SSD, 타겟 선택 |
| `src/motor.cpp` | `PanTilt` 인터페이스, pigpiod 서보 구현, dummy |
| `src/tracker.cpp` | PID, 화면 오차 → 각도 변화량, 소실 시 홈 복귀 |
| `src/app.cpp` | 메인 루프, 통계, 디버그 오버레이 |
| `src/config.cpp` | 기본값 + JSON 덮어쓰기, 알 수 없는 키 거부 |
| `src/log.cpp` | 작은 printf 로거 |

각 계층은 인터페이스로 분리돼 있어 교체가 쉽다.
예: `detector.cpp` 에 YOLO 백엔드를 추가하고 `makeDetector` 에 한 줄 넣으면 끝.

## 할 일

- **서보 캘리브레이션 도구** — 가동 범위와 회전 방향을 실측해 `config.json` 에
  반영하는 도구가 필요하다. 서보 연결 후 작성 예정.
- **in-process 인코딩** — 지금은 raw YUV420 을 파이프로 ffmpeg 에 넘긴다(41MB/s).
  libavcodec 으로 직접 인코딩하면 이 파이프와, 콜백에서의 프레임 복사가 모두
  사라진다. 측정상 복사 1.53% + 파이프분 → 3~5%p 정도의 이득이 예상된다.

## 알려진 제약

- MobileNet-SSD 는 320x240 입력에서 Pi 4 CPU 기준 대략 5~8 fps.
  더 빠르게 하려면 `detect.interval` 을 2~3 으로 올린다.
- 실행 중 ffmpeg 이 `Timestamps are unset in a packet` 경고를 낸다. raw H.264 를
  파이프로 받기 때문이며 동작에는 문제가 없다.
- MediaMTX 가 `RTP packets are too big (1460 > 1440)` 경고를 낸다. ffmpeg 4.3 의
  RTSP muxer 가 RTP 패킷 크기를 고정하고 있어 옵션으로 바꿀 수 없다(실측 확인).
  서버가 다시 쪼개 주며 비용은 무시할 수준이다.

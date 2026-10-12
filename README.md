# Auto Person Tracking Camera (camtracker)

라즈베리파이 4 + IMX219(Pi Camera v2) + MG90 서보 2개로
**RTSP 실시간 송출**과 **사람 추적 팬/틸트**를 수행한다.

- 유저스페이스 앱 — C++17. libcamera 직접 제어, MobileNet-SSD 검출, PID 추적
- 커널 모듈 — 서보 PWM 드라이버. 하드웨어 PWM, 비상정지 버튼 인터럽트, LED 트리거

## 측정으로 확인한 것

설계 결정마다 실측으로 근거를 남겼다. 틀렸던 첫 판단과 그것을 뒤집은 과정까지
[`docs/measurements/`](docs/measurements/) 에 있다.

| 주제 | 결과 | 문서 |
|---|---|---|
| 서보 펄스 지터 | hrtimer 로 만들면 부하 시 최대 122µs(약 11도) 튐 → 하드웨어 PWM 으로 바꿔 제거 | [jitter.md](docs/measurements/jitter.md) |
| 비상정지 지연 | 유저 폴링(1ms) 542µs·CPU 2.34% → 커널 IRQ 에서 직접 정지 45.7µs·CPU 0 | [button_latency.md](docs/measurements/button_latency.md) |
| PWM 생성 CPU | pigpiod 상주 +1.7%p → 커널 모듈은 측정 한계(±0.1%p) 아래 | [cpu.md](docs/measurements/cpu.md) |
| 검출 모델 | MobileNet-SSD 154ms vs YOLOv4-tiny(320) 262ms, 박스 안정성은 동일 → SSD 유지 | [model.md](docs/measurements/model.md) |
| 추적 로그 | 근거리에서 tilt 가 20도 튀는 원인(프레임에 잘린 박스) 규명 | [tracking.md](docs/measurements/tracking.md) |

## 현재 상태

| 기능 | 상태 |
|---|---|
| RTSP 송출 | 동작 확인 — 1280x720 H.264 30fps, 바운딩 박스 포함 |
| 사람 검출 | 동작 확인 — MobileNet-SSD(추론 약 155ms) + KCF 보간, 검출 루프 3.5~7 fps |
| 추적 (PID) | 동작 확인 — 지연 보상 포함. 중앙 수렴까지 약간 출렁임, 튜닝 여지 있음 |
| 서보 제어 | 동작 확인 — MG90 ×2, 가동범위 ±90도 실측 |
| 커널 서보 드라이버 | 동작 확인 — `/dev/servo0`, 하드웨어 PWM. pigpio 를 대체 |
| 비상정지 버튼 | 동작 확인 — 커널이 직접 정지. 정지 중 모든 명령 거부 |
| 상태 표시 LED | 동작 확인 — 커널이 트리거를 발행, 어느 LED 에 붙일지는 유저가 정함 |

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
 ffmpeg (h264_v4l2m2m)                       PID
   │  인코딩만. 디코딩 없음.                     │  "pan 1683us"
   ▼                                             ▼
 MediaMTX :8554 ──→ 시청자              /dev/servo0
                                               │
                                        [ 커널 모듈 servo.ko ]
                                               │  pwm_apply_state
                                        [ 칩의 PWM 회로 ] ──→ MG90 x2
                                               ▲
                                        버튼 GPIO17 ─ IRQ ─┘  비상정지
```

**펄스는 칩이 만든다.** 커널 모듈은 duty 값을 바꿀 뿐이고 CPU 는 파형에 관여하지
않는다. 그래서 부하가 걸려도 펄스폭이 흔들리지 않는다(hrtimer 로 만들 때는 부하 시
최대 11도까지 튀었다 — [jitter.md](docs/measurements/jitter.md)).

libcamera C++ API 로 카메라를 직접 연다. ISP 가 두 해상도를 하드웨어로 동시에
출력하므로 소프트웨어 축소가 없고, 검출용 프레임은 압축을 거치지 않는다.
외부 프로세스는 인코딩을 맡은 ffmpeg 하나뿐이다.

검출 좌표는 송출 프레임의 Y(밝기) 평면에 그려진다. 색 평면을 건드리지 않으므로
변환 비용이 붙지 않는다 — 박스는 흰색으로 보인다.

## 하드웨어 연결

| 대상 | BCM 핀 | 물리 핀 | 비고 |
|------|--------|---------|------|
| 팬(pan) 서보 신호   | GPIO12 | 32번 | 하드웨어 PWM0_0 |
| 틸트(tilt) 서보 신호 | GPIO13 | 33번 | 하드웨어 PWM0_1 |
| 서보 공통 GND       | —      | 30번 | 외부 5V 전원의 GND 와 묶는다 |
| 비상정지 버튼        | GPIO17 | 11번 | 반대쪽은 GND(9번). 내부 풀업을 커널 모듈이 켠다 |
| 상태 LED            | GPIO27 | 13번 | 저항 → LED → GND(14번) |

- **서보 핀은 GPIO12/13 으로 고정이다.** 칩의 하드웨어 PWM 채널이 이 핀에만 연결된다.
  (예전 pigpio 백엔드는 DMA 로 타이밍을 만들어 아무 핀이나 쓸 수 있었다.)
- 서보 전원(빨강)은 **파이 5V 핀이 아니라 별도 5V 공급**에 연결할 것.
  MG90S 는 스톨 시 700mA 이상을 먹어 파이가 리셋된다. GND 는 파이와 공통으로 묶는다.

## 빠른 시작

```bash
./scripts/install.sh          # 빌드 도구·OpenCV·libcamera·ffmpeg, MediaMTX
./scripts/download_model.sh   # MobileNet-SSD 가중치 (~23MB)
make                          # 빌드
make run                      # MediaMTX 까지 같이 실행
```

기본 설정(`motor.backend: kservo`)은 커널 모듈이 적재돼 있어야 서보가 움직인다
— 아래 [커널 서보 드라이버](#커널-서보-드라이버) 참고. 모듈이 없으면 경고를 찍고
모터만 끈 채로 송출과 검출은 계속한다.

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
make          # build/camtracker (바뀐 파일만 재컴파일)
make tools    # tools/servo_test, button_poll, button_wait
make clean    # 빌드 결과물 삭제
```

필요한 패키지: `g++`, `make`, `pkg-config`, `libopencv-dev`, `libcamera-dev`,
`pigpio`(`pigpiod_if2.h` 헤더 — `servo` 백엔드용). `scripts/install.sh` 가 설치한다.
`third_party/nlohmann/json.hpp` 는 저장소에 포함되어 있다.

## 커널 서보 드라이버

서보 펄스를 커널이 만든다. 유저스페이스는 `/dev/servo0` 에 한 줄 쓰는 게 전부다.

```bash
echo "pan 1500"   > /dev/servo0    # 펄스폭(µs) — 기본 단위
echo "pan 30deg"  > /dev/servo0    # 정수 각도 (커널이 환산)
echo "pan off"    > /dev/servo0    # 펄스 중단 — 서보가 힘을 놓는다
cat /dev/servo0                    # 현재 상태
```

`config.json` 의 `motor.backend` 를 `kservo` 로 두면 `camtracker` 가 이 경로를 쓴다.
커널은 µs 만 받고, 서보마다 다른 보정값(`min_pulse_ms`/`max_pulse_ms`)은
`config.json` 이 갖는다.

### 준비

`/boot/config.txt` 에 다음을 반영하고 재부팅한다.

```
#dtparam=audio=on
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
dtoverlay=servo-pwm
dtoverlay=servo-led
```

| 줄 | 이유 |
|---|---|
| `#dtparam=audio=on` | 아날로그 오디오가 같은 PWM 채널을 쓰므로 끈다 |
| `pwm-2chan` | GPIO12/13 을 칩의 PWM 회로에 붙이고(ALT0) PWM 노드를 켠다 |
| `servo-pwm` | "servo 장치가 PWM 채널 0·1 을 쓴다" 는 선언 — 이 저장소의 오버레이 |
| `servo-led` | GPIO27 의 LED 를 커널 LED 장치로 등록 — 이 저장소의 오버레이 |

```bash
make -C kernel/servo                                     # servo.ko
dtc -@ -I dts -O dtb -o kernel/servo/servo-pwm.dtbo kernel/servo/servo-pwm.dts
dtc -@ -I dts -O dtb -o kernel/servo/servo-led.dtbo kernel/servo/servo-led.dts
sudo cp kernel/servo/*.dtbo /boot/overlays/
sudo cp kernel/servo/99-servo.rules /etc/udev/rules.d/   # gpio 그룹에 권한
sudo reboot

sudo insmod kernel/servo/servo.ko
dmesg | grep servo          # "PWM 2채널 확보 — 펄스 생성은 하드웨어 PWM"
sudo kernel/servo/test_servo.sh   # 실제 서보로 동작·거부 입력 확인
```

`servo-pwm` 오버레이가 없으면 모듈은 hrtimer 로 펄스를 만드는 경로로 동작한다.
두 경로를 남겨 둔 이유는 비교 측정 때문이다.

### 비상정지

버튼(GPIO17)을 누르면 커널이 직접 서보를 멈춘다. **유저 프로그램이 하나도 떠 있지
않아도 동작한다.** 그리고 정지 중에는 `/dev/servo0` 에 오는 모든 명령이 거부된다
(`EBUSY`) — 서보로 가는 길이 하나뿐이라 커널이 문지기가 된다. 한 번 더 누르면 해제되고
원래 펄스폭으로 돌아간다.

```bash
cat /sys/class/servo/servo0/estop
# mode kernel  method thread  path thread  engaged 0  stops 6  avg 45.7us  max 61.1us

echo pinmux > /sys/class/servo/servo0/estop   # 상반부에서 핀을 PWM 에서 뗀다   8.0µs
echo thread > /sys/class/servo/servo0/estop   # 하반부에서 정식 API 로 끈다    45.7µs (기본)
echo clear  > /sys/class/servo/servo0/estop   # 정지 해제 + 통계 초기화
```

하드웨어 PWM 을 끄는 `pwm_apply_state()` 는 잠들 수 있어 인터럽트 핸들러 안에서
부를 수 없다. 그래서 두 방법을 만들어 쟀고, 규약을 지키는 `thread` 를 기본으로 했다
— [button_latency.md](docs/measurements/button_latency.md).

### 상태 표시 LED

드라이버는 "정지 중 / 동작 중" 이라는 **사실만 트리거로 발행**하고, 어느 불로
보여줄지는 밖에서 정한다.

```bash
echo servo-estop > /sys/class/leds/estop/trigger    # GPIO27 LED 가 정지 상태를 따른다
echo servo-run   > /sys/class/leds/<이름>/trigger   # 동작 중 표시 (LED 를 추가했을 때)
```

LED 를 늘리거나 핀을 옮겨도 드라이버 코드는 그대로다. LED 추가는
`kernel/servo/servo-led.dts` 에 블록 하나를 복사하면 된다.

### 상태 파일

| 경로 | 내용 |
|------|------|
| `/sys/class/servo/servo0/estop` | 비상정지 상태·방법·지연 통계, 모드 전환 |
| `/sys/class/servo/servo0/button` | 버튼 인터럽트·채터링 통계 |
| `/sys/class/servo/servo0/press` | 마지막 눌림 번호와 커널 시각 (폴링 지연 측정의 기준시계) |
| `/sys/class/servo/servo0/level` | 버튼 핀의 현재 값 (모듈이 핀을 쥐고 있을 때 폴링용) |
| `/sys/class/servo/servo0/jitter` | hrtimer 경로의 타이머 지터 (PWM 경로에서는 무의미) |
| `/dev/button0` | 버튼이 눌릴 때까지 잠드는 `read()` |

## 캘리브레이션

서보마다 펄스폭 범위와 회전 방향이 다르므로 실측해 `config.json` 에 넣는다.

```bash
echo "pan 1500" > /dev/servo0    # 커널 드라이버로 펄스폭을 직접 조작해 범위를 본다
```

또는 대화형 도구 `tools/servo_test` 를 쓴다. pigpiod 로 펄스를 내므로 커널 모듈을
먼저 내린다.

```bash
sudo rmmod servo && sudo systemctl start pigpiod
make tools && ./tools/servo_test
```

각도가 아니라 펄스폭(µs)을 다루는 이유는, 각도 변환에 쓰이는
`min_pulse_ms`/`max_pulse_ms` 자체가 지금 확인하려는 대상이기 때문이다. 시작 시
양쪽을 1500µs(중앙)로 보낸다 — 현재 각도를 알 수 없으므로 어느 쪽 끝에서든
안쪽으로 들어오는 중앙이 가장 안전하다.

**방향은 몸이 아니라 화면 기준으로 판단할 것.** "내가 왼쪽에 섰다"는 카메라 영상에서
오른쪽일 수 있다. 확실한 방법은 `--log-level DEBUG` 로 추적 로그를 보고 `err` 이
0 으로 줄어드는지 확인하는 것이다:

```
box=(64,120)/320x240  err=(-0.60,+0.00)  ff=(+0.0,+0.0)  d=(-4.0,+0.0)  now=(+0.0,+0.0)
```

`err` 이 커지면 서보가 반대로 도는 것 → 해당 축의 `invert` 를 뒤집는다.

## 설정 (`config.json`)

적힌 키만 기본값(`src/config.hpp`)을 덮어쓴다. 전체 값은
`./build/camtracker --dump-config` 로 확인.

| 키 | 의미 |
|----|------|
| `camera.width/height/fps/bitrate` | 송출 해상도·비트레이트 |
| `stream.rtsp_url` | 푸시할 RTSP 주소 |
| `detect.backend` | `ssd` \| `none` |
| `detect.width/height/fps` | 검출 파이프라인 해상도·프레임레이트 상한 |
| `detect.interval` | N 프레임마다 1회 검출 (CPU 절약) |
| `detect.tracker`, `redetect_interval` | 검출 사이를 메울 트래커(`kcf` \| `csrt` \| `none`), N 프레임마다 실제 검출로 보정 |
| `motor.backend` | `kservo`(커널 모듈) \| `syspwm`(sysfs PWM, 비교용) \| `servo`(pigpiod) \| `dummy` |
| `motor.min_pulse_ms/max_pulse_ms` | -90도/+90도에 해당하는 펄스폭. MG90S 실측 0.5~2.5ms |
| `motor.pan_min/max`, `tilt_min/max` | 기구적 가동 범위(deg). 실측 ±90 |
| `motor.pan_invert`, `tilt_invert` | 서보가 반대로 돌 때만 `true` |
| `motor.pan_pin`, `tilt_pin` | `servo` 백엔드만 쓴다. 커널 경로는 GPIO12/13 고정 |
| `track.deadzone` | 화면 중앙 이 비율 안에서는 움직이지 않음 |
| `track.pan_pid.kp/ki/kd`, `max_step` | 추적 응답성. 떨리면 `kp`↓ `kd`↑ |
| `track.compensate_latency`, `latency_ms` | 이미 보냈지만 아직 영상에 안 나타난 명령을 오차에서 뺀다 |
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
                   [--log-level DEBUG|INFO|WARNING|ERROR]
                   [--dump-config] [--check-model]
```

## 부팅 시 자동 실행

```bash
./systemd/install-services.sh
journalctl -fu camtracker
```

커널 모듈도 부팅할 때 올리려면:

```bash
sudo mkdir -p /lib/modules/$(uname -r)/extra
sudo cp kernel/servo/servo.ko /lib/modules/$(uname -r)/extra/
sudo depmod -a
echo servo | sudo tee /etc/modules-load.d/servo.conf
```

`depmod` 를 빼먹으면 `modprobe` 가 모듈을 찾지 못한다. 부팅 때는 모듈이 PWM 칩
드라이버보다 먼저 올라올 수 있는데, 그 경우에도 PWM 이 준비되는 순간 probe 가
불려 하드웨어 PWM 으로 전환된다.

## 저장소 구성

| 경로 | 역할 |
|------|------|
| `src/camera.cpp` | libcamera 직접 제어, 듀얼 스트림, 박스 오버레이, 인코더 파이프 |
| `src/detector.cpp` | `Detector` 인터페이스, MobileNet-SSD, KCF 보간, 타겟 선택 |
| `src/motor.cpp` | `PanTilt` 인터페이스 — `kservo` / `syspwm` / `servo`(pigpio) / `dummy` |
| `src/tracker.cpp` | PID, 지연 보상, 화면 오차 → 각도 변화량, 소실 시 홈 복귀 |
| `src/app.cpp` | 메인 루프, 비상정지 연동, 통계, 디버그 스냅샷 |
| `src/config.cpp` | 기본값 + JSON 덮어쓰기, 알 수 없는 키 거부 |
| `src/log.cpp` | 작은 printf 로거 |
| `kernel/servo/servo.c` | 커널 모듈 — 캐릭터 디바이스, 하드웨어 PWM, 버튼 IRQ, LED 트리거 |
| `kernel/servo/*.dts` | 디바이스 트리 오버레이 — PWM 채널 선언, LED 등록 |
| `tools/` | 서보 캘리브레이션, 비상정지 지연 측정(폴링/IRQ), libcamera 최소 예제 |
| `scripts/` | 설치, 모델 다운로드, CPU·추적 측정 스크립트 |
| `docs/measurements/` | 측정 기록과 원본 로그 |

각 계층은 인터페이스로 분리돼 있어 교체가 쉽다.
예: `detector.cpp` 에 YOLO 백엔드를 추가하고 `makeDetector` 에 분기 하나를 넣으면 된다.

## 할 일

- **근거리에서 tilt 가 튄다** — 사람이 프레임보다 크면 검출 박스가 위아래로 잘리고,
  잘린 박스의 중심은 카메라가 어디를 보든 화면 중앙이라 세로 오차가 0 으로 고정된다.
  간헐적 부분 검출이 겹치면 제어기가 한 방향으로 20도를 몰아간다. 원인은 규명했고
  근본 해결(거리·화각 또는 검출 개선)은 남아 있다 — [tracking.md](docs/measurements/tracking.md).
- **in-process 인코딩** — 지금은 raw YUV420 을 파이프로 ffmpeg 에 넘긴다(41MB/s).
  libavcodec 으로 직접 인코딩하면 이 파이프와, 콜백에서의 프레임 복사가 모두
  사라진다. 측정상 복사 1.53% + 파이프분 → 3~5%p 정도의 이득이 예상된다.
- **비상정지가 fail-safe 가 아니다** — 풀업 + 눌렀을 때 GND 배선이라 선이 끊어지면
  정지가 걸리지 않는다. 산업용처럼 평소 닫힌 접점(NC)으로 바꿔야 한다.

## 알려진 제약

- 검출 지연(촬영 → 박스)이 약 220ms 다. 모델 추론만 약 155ms 라, 사이 프레임은
  KCF 트래커로 메우고 지연은 `track.compensate_latency` 로 보상한다.
- 실행 중 ffmpeg 이 `Timestamps are unset in a packet` 경고를 낸다. raw H.264 를
  파이프로 받기 때문이며 동작에는 문제가 없다.
- MediaMTX 가 `RTP packets are too big (1460 > 1440)` 경고를 낸다. ffmpeg 4.3 의
  RTSP muxer 가 RTP 패킷 크기를 고정하고 있어 옵션으로 바꿀 수 없다(실측 확인).
  서버가 다시 쪼개 주며 비용은 무시할 수준이다.

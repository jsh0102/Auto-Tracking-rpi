# 측정 자료

커널 서보 드라이버(`kernel/servo/`)를 만들면서 실측한 값들. 하드웨어가 있어야만
얻을 수 있는 자료라 별도로 보관한다.

| 파일 | 내용 |
| --- | --- |
| [jitter.md](jitter.md) | hrtimer 지터 4개 조건·히스토그램·원인 추적, **하드웨어 PWM 검증** |
| [cpu.md](cpu.md) | pigpio vs 커널 모듈 CPU 사용률 |
| [tracking.md](tracking.md) | 실제 추적 로그 분석 |
| [model.md](model.md) | 검출 모델 비교 — MobileNet-SSD vs YOLOv4-tiny |
| [button_latency.md](button_latency.md) | 비상정지 버튼 — 폴링 vs 인터럽트 지연·CPU |
| `tracking_kservo.log` | 위 분석의 원본 로그 (290줄) |
| `tracking_syspwm.log` | 하드웨어 PWM(syspwm)으로 추적한 원본 로그 (267줄) |

## 측정 환경

```
Raspberry Pi 4B / Bullseye / 커널 6.1.21-v8+ (CONFIG_PREEMPT=y, CONFIG_HZ=250)
IMX219 CSI 카메라, MG90 서보 2개 (팬 GPIO12, 틸트 GPIO13)
서보 펄스 0.5~2.5ms / 주기 20ms(50Hz) — 실측으로 정한 값
```

펄스를 만드는 주체가 셋이고, 측정은 그 셋을 갈아 끼우며 한다.

```
servo    pigpiod 가 /dev/mem + DMA 로 만든다        (유저스페이스, 커널 우회)
kservo   커널 모듈이 hrtimer 콜백으로 핀을 흔든다   (kernel/servo/servo.ko)
syspwm   칩 안의 PWM 회로가 만든다                  (dtoverlay=pwm-2chan 필요)
```

백엔드를 오갈 때 두 가지를 주의한다.

- **pigpiod 를 SIGKILL 하면** DMA·하드웨어 상태가 남아 다음에 시작해도 펄스가 나가지
  않는다. 복구는 재부팅뿐. (커널 모듈은 `rmmod` 해도 커널이 정리한다)
- 핀이 HIGH 로 남을 수 있다 -> `raspi-gpio set 12 op dl`.
  반대로 하드웨어 PWM 으로 되돌릴 때는 `raspi-gpio set 12 a0` (ALT0)

## 재현 방법

```bash
# 지터 — 모듈이 직접 잰다
sudo insmod kernel/servo/servo.ko
echo "pan 1500" > /dev/servo0
echo 0 | sudo tee /sys/class/servo/servo0/jitter    # 통계 초기화
cat /sys/class/servo/servo0/jitter                  # 1~2분 뒤

# CPU — 네 조건을 60초씩, 기준선을 두 번 재서 오차범위를 만든다
sudo ./scripts/bench_pwm.sh

# 추적 로그
SECS=40 ./scripts/tune_capture.sh docs/measurements/tracking_kservo.log

# 하드웨어 PWM — 먼저 /boot/config.txt 에 두 줄이 필요하다 (재부팅)
#   dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
#   #dtparam=audio=on          <- 아날로그 오디오가 같은 PWM 채널을 쓴다
ls /sys/class/pwm/pwmchip0                     # 채널이 생겼는지
raspi-gpio get 12,13                           # func=PWM0_0 / PWM0_1 인지

# 손으로 파형 내보내기 (sudo 불필요 — 파일이 root:gpio 그룹 쓰기)
P=/sys/class/pwm/pwmchip0
echo 0 > $P/export; echo 20000000 > $P/pwm0/period
echo 1500000 > $P/pwm0/duty_cycle; echo 1 > $P/pwm0/enable

# ① 지터만 분리 — 명령을 고정해 두고 부하만 건다 (사람 불필요)
./build/camtracker --no-track &                # 부하. 모터는 건드리지 않는다
echo 1500000 > $P/pwm0/duty_cycle              # 30초간 그대로 두고 눈으로 본다

# ② 실제 추적 — config.json 의 motor.backend 를 syspwm 으로
SECS=40 ./scripts/tune_capture.sh docs/measurements/tracking_syspwm.log

# 버튼 폴링 기준선 (조건당 20초, 버튼을 몇 번 눌러야 함)
./tools/button_poll --interval 10ms --seconds 20
./tools/button_poll --busy --seconds 20
```

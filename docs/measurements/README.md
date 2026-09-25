# 측정 자료

커널 서보 드라이버(`kernel/servo/`)를 만들면서 실측한 값들. 하드웨어가 있어야만
얻을 수 있는 자료라 별도로 보관한다.

| 파일 | 내용 |
| --- | --- |
| [jitter.md](jitter.md) | hrtimer 타이머 지터. 4개 조건, 히스토그램, 원인 추적 |
| [cpu.md](cpu.md) | pigpio vs 커널 모듈 CPU 사용률 |
| [tracking.md](tracking.md) | 실제 추적 로그 분석 |
| `tracking_kservo.log` | 위 분석의 원본 로그 (290줄) |

## 측정 환경

```
Raspberry Pi 4B / Bullseye / 커널 6.1.21-v8+ (CONFIG_PREEMPT=y, CONFIG_HZ=250)
IMX219 CSI 카메라, MG90 서보 2개 (팬 GPIO12, 틸트 GPIO13)
서보 펄스 0.5~2.5ms / 주기 20ms(50Hz) — 실측으로 정한 값
```

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
```

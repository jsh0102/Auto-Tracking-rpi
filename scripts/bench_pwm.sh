#!/usr/bin/env bash
# PWM 생성 방식별 CPU 사용률 비교 — pigpio(DMA) vs 커널 모듈(hrtimer)
#
#   sudo ./scripts/bench_pwm.sh          기본 60초씩
#   sudo SECS=120 ./scripts/bench_pwm.sh 더 길게
#
# 기준선을 처음과 끝에 두 번 잰다. 두 값의 차이가 곧 이 측정의 오차범위다.
# 그걸 모르면 "+0.3%p" 가 의미 있는 값인지 잡음인지 판단할 수 없다.
#
# 세 조건을 같은 시간만큼 재고 표로 낸다. 기준선을 빼야 "펄스 생성이 쓰는 CPU" 가
# 나온다 — Pi 는 가만히 둬도 몇 %를 쓴다.
#
# 사람은 필요 없다. 서보는 중앙(1500us)에 고정된 채로 있는다.

set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MOD="$ROOT/kernel/servo/servo.ko"
DEV=/dev/servo0
SECS="${SECS:-60}"
PAN_GPIO=12
TILT_GPIO=13

die() { echo "오류: $*" >&2; exit 1; }

# ── CPU 측정 ────────────────────────────────────────────────
# /proc/stat 의 누적 시간을 두 번 읽어 차이를 본다.
#   총합 = 모든 항목,  유휴 = idle + iowait
snapshot() {
	awk '/^cpu /{idle=$5+$6; total=0; for(i=2;i<=NF;i++) total+=$i;
	     print total, idle}' /proc/stat
}

# measure <초>  ->  "12.3" (사용률 %)
measure() {
	local t0 i0 t1 i1 dt di per_mille
	read -r t0 i0 < <(snapshot)
	sleep "$1"
	read -r t1 i1 < <(snapshot)
	dt=$((t1 - t0))
	di=$((i1 - i0))
	[ "$dt" -gt 0 ] || { echo "0.0"; return; }
	# 커널에 소수점이 없던 것과 같은 방법. 1000배 해서 정수로 계산한다.
	per_mille=$(( (dt - di) * 1000 / dt ))
	echo "$((per_mille / 10)).$((per_mille % 10))"
}

# ── pigpiod 제어 ────────────────────────────────────────────
# 이 패키지의 서비스 파일은 ExecStop 이 강제 kill 이라 systemctl stop 이
# 90초까지 걸린다. 측정 스크립트에서 기다릴 이유가 없어 바로 SIGKILL 한다.
stop_pigpiod() {
	if pgrep -x pigpiod > /dev/null; then
		# 죽이기 전에 펄스부터 끊는다. 그냥 죽이면 핀이 HIGH 로 남아
		# 서보가 계속 힘을 쓴다.
		pigs s $PAN_GPIO 0  2>/dev/null || true
		pigs s $TILT_GPIO 0 2>/dev/null || true
		sleep 1
		systemctl kill -s SIGKILL pigpiod 2>/dev/null || true
		pkill -9 pigpiod 2>/dev/null || true
	fi
	for _ in $(seq 25); do
		pgrep -x pigpiod > /dev/null || return 0
		sleep 0.2
	done
	die "pigpiod 를 멈추지 못했습니다"
}

start_pigpiod() {
	systemctl start pigpiod 2>/dev/null || die "pigpiod 를 띄우지 못했습니다"
	for _ in $(seq 25); do
		pgrep -x pigpiod > /dev/null && break
		sleep 0.2
	done
	sleep 1		# 데몬이 DMA 를 준비할 시간
}

# ── 커널 모듈 제어 ──────────────────────────────────────────
load_module() {
	lsmod | grep -q "^servo " || insmod "$MOD" || die "모듈 적재 실패"
	[ -e "$DEV" ] || die "$DEV 가 없습니다"
}

unload_module() {
	if lsmod | grep -q "^servo "; then
		[ -e "$DEV" ] && { echo "pan off" > "$DEV"; echo "tilt off" > "$DEV"; }
		sleep 1
		rmmod servo || die "모듈 제거 실패 (누가 $DEV 를 열고 있나 확인)"
	fi
}

cleanup() {
	echo
	echo "==> 정리"
	stop_pigpiod
	if lsmod | grep -q "^servo " && [ -e "$DEV" ]; then
		echo "pan off"  > "$DEV" 2>/dev/null || true
		echo "tilt off" > "$DEV" 2>/dev/null || true
		echo "    펄스 중단"
	fi
}
trap cleanup EXIT INT TERM

# ── 사전 확인 ───────────────────────────────────────────────
[ "$(id -u)" -eq 0 ] || die "sudo 로 실행하세요:  sudo $0"
[ -f "$MOD" ]        || die "모듈이 없습니다: $MOD  (make -C $(dirname "$MOD"))"
command -v pigs > /dev/null || die "pigs 가 없습니다 (pigpio 패키지)"
pgrep -x camtracker > /dev/null && die "camtracker 가 돌고 있습니다. 종료하세요"

GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo "?")
if [ "$GOV" != "performance" ]; then
	echo "!! governor 가 '$GOV' 입니다. 클럭이 오르내리면 비교가 흔들립니다."
	echo "   권장:  echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor"
	echo
fi

echo "==> 조건마다 ${SECS}초씩, 총 $((SECS * 3 + 15))초 정도 걸립니다"
echo "    governor=$GOV  서보는 중앙(1500us)에 고정됩니다"
echo

# ── ① 기준선 (처음) ────────────────────────────────────────
echo "[1/4] 기준선 — 펄스 없음"
stop_pigpiod
unload_module
sleep 2
BASE_A=$(measure "$SECS")
echo "      $BASE_A%"

# ── ② pigpio ───────────────────────────────────────────────
echo "[2/4] pigpio — 데몬 + DMA"
start_pigpiod
pigs s $PAN_GPIO 1500  || die "pigs 전송 실패"
pigs s $TILT_GPIO 1500 || die "pigs 전송 실패"
sleep 2
PIGPIO=$(measure "$SECS")
echo "      $PIGPIO%"
stop_pigpiod

# ── ③ 커널 모듈 ────────────────────────────────────────────
echo "[3/4] 커널 모듈 — hrtimer"
load_module
echo "pan 1500"  > "$DEV"
echo "tilt 1500" > "$DEV"
sleep 2
KERNEL=$(measure "$SECS")
echo "      $KERNEL%"

# ── ④ 기준선 (끝) ──────────────────────────────────────────
# ① 과 똑같은 조건으로 한 번 더. 같은 조건인데 값이 다르면 그만큼이 잡음이다.
echo "[4/4] 기준선 다시 — 오차범위 확인"
unload_module
sleep 2
BASE_B=$(measure "$SECS")
echo "      $BASE_B%"
load_module

# ── 결과 ────────────────────────────────────────────────────
# 소수 한 자리를 10배 정수로 바꿔 계산한다 (12.3 -> 123)
to_i()   { local v=$1; echo $(( ${v%.*} * 10 + ${v#*.} )); }
signed() {
	local v=$1
	if [ "$v" -lt 0 ]; then echo "-$(( (-v) / 10 )).$(( (-v) % 10 ))"
	else                    echo "+$(( v / 10 )).$(( v % 10 ))"; fi
}
plain()  { echo "$(( $1 / 10 )).$(( $1 % 10 ))"; }

AI=$(to_i "$BASE_A"); BI=$(to_i "$BASE_B")
PI=$(to_i "$PIGPIO");  KI=$(to_i "$KERNEL")

NOISE=$(( AI > BI ? AI - BI : BI - AI ))   # 같은 조건 두 번의 차이 = 오차범위
BAVG=$(( (AI + BI) / 2 ))
PD=$(( PI - BAVG ))
KD=$(( KI - BAVG ))

# 차이가 오차범위보다 크면 의미 있는 값이다.
verdict() {
	local a=$1
	[ "$a" -lt 0 ] && a=$(( -a ))
	if [ "$NOISE" -eq 0 ]; then echo "유의미"
	elif [ "$a" -gt "$NOISE" ]; then echo "유의미"
	else echo "오차범위 안"; fi
}

echo
echo "──────────────────────────────────────────────────────────"
printf " %-18s %7s   %9s   %s\n" "조건" "CPU" "기준선대비" "판정"
echo "──────────────────────────────────────────────────────────"
printf " %-18s %6s%%   %9s   %s\n" "① 기준선(처음)"  "$BASE_A" "—" ""
printf " %-18s %6s%%   %9s   %s\n" "④ 기준선(끝)"    "$BASE_B" "—" ""
echo "──────────────────────────────────────────────────────────"
printf " %-18s %6s%%   %8s%%p   %s\n" "② pigpio(DMA)"   "$PIGPIO" "$(signed $PD)" "$(verdict $PD)"
printf " %-18s %6s%%   %8s%%p   %s\n" "③ 커널(hrtimer)" "$KERNEL" "$(signed $KD)" "$(verdict $KD)"
echo "──────────────────────────────────────────────────────────"
echo " 오차범위 ±$(plain $NOISE)%p   (같은 조건을 두 번 잰 차이)"
echo " 측정: 조건당 ${SECS}초, governor=$GOV, 서보 2채널 50Hz"
echo
echo " '오차범위 안' = 그 방식이 쓰는 CPU 가 이 측정으로는 잡히지 않는다는 뜻"

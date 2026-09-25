#!/bin/bash
# C-4 서보 동작 확인 — 커널 드라이버(servo.ko)로 실제 서보를 움직인다.
#
#   sudo ./test_servo.sh
#
# 카메라와 서보가 다른 방에 있으므로 시작 전에 걸어갈 시간을 준다.
# Ctrl+C 로 중단해도 펄스를 끊고 모듈을 내린다 — 안 그러면 서보가 계속
# 힘을 준 채로 남아 발열한다.

set -u

MOD_PATH="$(cd "$(dirname "$0")" && pwd)/servo.ko"
MOD_NAME=servo
DEV=/dev/servo0
WALK_SECONDS=10		# 걸어갈 시간 (서보는 바로 옆방)
HOLD=3			# 각 자세를 유지하는 시간

die() { echo "오류: $*" >&2; exit 1; }

# trap 으로 걸어두면 정상 종료·Ctrl+C·kill 어느 쪽이든 여기를 지나간다.
cleanup() {
	echo
	echo "==> 정리"
	if [ -e "$DEV" ]; then
		echo "pan 0"  > "$DEV" 2>/dev/null
		echo "tilt 0" > "$DEV" 2>/dev/null
		echo "    펄스 중단 (서보가 힘을 놓음)"
	fi
	if lsmod | grep -q "^$MOD_NAME "; then
		rmmod "$MOD_NAME" 2>/dev/null && echo "    모듈 제거"
	fi
}
trap cleanup EXIT INT TERM

# ── 사전 확인 ────────────────────────────────────────────────
[ "$(id -u)" -eq 0 ] || die "sudo 로 실행하세요:  sudo $0"
[ -f "$MOD_PATH" ]   || die "모듈이 없습니다: $MOD_PATH
       먼저 빌드하세요:  make -C $(dirname "$MOD_PATH")"

if systemctl is-active --quiet pigpiod; then
	die "pigpiod 가 돌고 있습니다. 같은 핀을 두 쪽이 만지면 안 됩니다.
       sudo systemctl stop pigpiod"
fi

if lsmod | grep -q "^$MOD_NAME "; then
	echo "==> 이미 적재돼 있어 먼저 제거합니다"
	rmmod "$MOD_NAME" || die "제거 실패 (누가 $DEV 를 열고 있는지 확인)"
fi

# ── 적재 ────────────────────────────────────────────────────
echo "==> 모듈 적재"
insmod "$MOD_PATH" || die "적재 실패 — sudo dmesg | tail 로 원인을 보세요"
[ -e "$DEV" ]      || die "$DEV 가 생기지 않았습니다"
echo "    $(ls -l "$DEV")"

# ── 걸어갈 시간 ──────────────────────────────────────────────
echo
echo "==> ${WALK_SECONDS}초 뒤 시작합니다. 서보 앞으로 이동하세요."
for ((i = WALK_SECONDS; i > 0; i--)); do
	printf "\r    %2d초 남음..." "$i"
	sleep 1
done
printf "\r    시작합니다!      \n"

# ── 동작 ────────────────────────────────────────────────────
# send <채널> <값> <설명>
#   값은 "0deg" / "1500" / "off" 중 아무 형식이나 된다.
send() {
	printf "    %-22s  %s %s\n" "$3" "$1" "$2"
	echo "$1 $2" > "$DEV" || die "write 실패"
	sleep "$HOLD"
}

# 커널이 지금 무엇을 쏘고 있는지 읽어 본다 (C-5 에서 추가된 read).
show_state() {
	echo "    ── cat $DEV ──"
	sed 's/^/       /' "$DEV"
}

echo
echo "── 팬 (GPIO12) ── 각도로 지정"
send pan 0deg   "가운데"
show_state
send pan -45deg "한쪽으로"
send pan 45deg  "반대쪽으로"
send pan 0deg   "가운데로 복귀"
send pan off    "힘 놓기"
show_state

echo
echo "── 틸트 (GPIO13) ── 마이크로초로 지정"
send tilt 1500 "가운데"
send tilt 1200 "한쪽으로"
send tilt 1800 "반대쪽으로"
send tilt 1500 "가운데로 복귀"
send tilt off  "힘 놓기"

echo
echo "── 거부되어야 하는 입력 ──"
for bad in "pan 200deg" "pan 3000us" "pan 0" "pan xyz" "elbow 30deg"; do
	if echo "$bad" > "$DEV" 2>/dev/null; then
		echo "    !! 통과됐습니다 (막혔어야 함): $bad"
	else
		echo "    거부 OK: $bad"
	fi
done

echo
echo "==> 동작 끝. 아래를 확인해 주세요."
echo "     1) 서보가 실제로 움직였는가"
echo "     2) 한 자세에서 3초간 가만히 있었는가 (떨림 = 타이밍 문제)"
echo "     3) '힘 놓기' 후 손으로 돌려지는가 (펄스 중단이 되는가)"
echo "     4) 각도 지정과 마이크로초 지정이 같은 범위로 움직였는가"

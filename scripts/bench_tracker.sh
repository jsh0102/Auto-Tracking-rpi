#!/usr/bin/env bash
# 검출 보간 효과 측정 — tracker=none 과 kcf 를 같은 조건으로 비교한다.
# ★ 측정 내내 카메라 앞에 사람이 있어야 한다. 대상이 없으면 추적할 것도 없다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
TMP=$(mktemp -d)
MTX_PID=""
cleanup() {
  [[ -n "$MTX_PID" ]] && kill "$MTX_PID" 2>/dev/null || true
  rm -rf "$TMP"
}
trap cleanup EXIT INT TERM

# RTSP 서버가 없으면 띄운다. 두 측정 구간 내내 살려둔다.
if ! ss -ltn 2>/dev/null | grep -q ':8554 '; then
  ./bin/mediamtx mediamtx.yml >/dev/null 2>&1 &
  MTX_PID=$!
  sleep 1
fi
IP=$(hostname -I | awk '{print $1}')

cpu() {  # cpu <초> — /proc/stat 기준 시스템 전체 사용률
  read_stat() { awk '/^cpu /{i=$5+$6; t=0; for(n=2;n<=NF;n++) t+=$n; print t, i}' /proc/stat; }
  read a1 i1 < <(read_stat); sleep "$1"; read a2 i2 < <(read_stat)
  awk -v a1=$a1 -v i1=$i1 -v a2=$a2 -v i2=$i2 'BEGIN{d=a2-a1; di=i2-i1; printf "%.1f", (d-di)*100.0/d}'
}

set_tracker() {
  python3 -c "
import json,pathlib
p=pathlib.Path('config.json'); d=json.loads(p.read_text())
d['detect']['tracker']='$1'
p.write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n')"
}

LEAD="${LEAD:-20}"
cat <<MSG
화면 확인:  http://${IP}:8889/cam

총 ${LEAD}초 준비 + 약 55초 측정 = 약 $((LEAD + 55))초 걸립니다.

  1) Enter 를 누르면 ${LEAD}초 카운트다운이 시작됩니다
  2) 그 사이에 카메라 앞으로 이동하세요
  3) 이후 약 55초간 계속 화면에 잡혀 있어야 합니다 (상반신 이상)
     중간에 프로그램이 한 번 재시작됩니다 — 그대로 계세요

MSG
read -rp "준비되면 Enter..." _
for ((i=LEAD; i>0; i--)); do printf "\r  카메라 앞으로 이동하세요... %2d초  " "$i"; sleep 1; done
printf "\r  측정 시작                      \n\n"

for T in none kcf; do
  set_tracker "$T"
  LIBCAMERA_LOG_LEVELS=*:ERROR ./build/camtracker --log-level INFO \
      >"$TMP/$T.log" 2>&1 &
  PID=$!
  sleep 12                       # 워밍업
  C=$(cpu 12)
  kill $PID 2>/dev/null || true; wait $PID 2>/dev/null || true; sleep 2

  HIT=$(grep -c "타겟 있음" "$TMP/$T.log" || true)
  MISS=$(grep -c "타겟 없음" "$TMP/$T.log" || true)
  FPS=$(grep -oE "[0-9.]+ fps" "$TMP/$T.log" | tail -2 | head -1)
  printf "  %-5s  CPU %5s%%   %-9s  타겟 있음 %s / 없음 %s\n" "$T" "$C" "$FPS" "$HIT" "$MISS"
done

set_tracker kcf
echo
echo "※ '타겟 없음' 이 많으면 측정이 무의미합니다. 다시 하세요."

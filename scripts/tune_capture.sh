#!/usr/bin/env bash
# PID 튜닝용 측정. 송출을 켜둬서 브라우저로 자기 위치를 보면서 할 수 있다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-/tmp/tune_$(date +%H%M%S).log}"
SECS="${SECS:-25}"
MEDIAMTX="$ROOT/bin/mediamtx"

cd "$ROOT"

# RTSP 서버가 없으면 같이 띄운다
MTX_PID=""
cleanup() { [[ -n "$MTX_PID" ]] && kill "$MTX_PID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
if ! ss -ltn 2>/dev/null | grep -q ':8554 '; then
  "$MEDIAMTX" "$ROOT/mediamtx.yml" >/dev/null 2>&1 &
  MTX_PID=$!
  sleep 1
fi

IP=$(hostname -I | awk '{print $1}')
cat <<MSG
==> 브라우저에서 화면 확인:  http://${IP}:8889/cam

    1) 화면에 초록 박스가 뜨는지 먼저 확인하세요 (안 뜨면 검출이 안 되는 것)
    2) 박스가 뜨면 화면 '한쪽 끝' 으로 이동해서 가만히 계세요
    3) ${SECS}초 측정합니다

MSG
read -rp "준비되면 Enter..." _

LIBCAMERA_LOG_LEVELS=*:ERROR timeout "$SECS" \
  ./build/camtracker --log-level DEBUG 2>&1 \
  | grep --line-buffered -E "box=|trk |DET |소실" | tee "$OUT"

echo
echo "==> 저장: $OUT   ($(wc -l < "$OUT") 줄)"

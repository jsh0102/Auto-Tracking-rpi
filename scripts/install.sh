#!/usr/bin/env bash
# 의존성 설치: libcamera-apps, ffmpeg, pigpiod, MediaMTX
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MEDIAMTX_DIR="${MEDIAMTX_DIR:-/opt/mediamtx}"

log() { printf '\033[1;32m==>\033[0m %s\n' "$*"; }

log "APT 패키지 설치"
sudo apt-get update
sudo apt-get install -y \
  libcamera-apps ffmpeg pigpio python3-gpiozero python3-numpy curl ca-certificates

log "pigpiod 활성화 (서보 PWM 지터 제거용)"
sudo systemctl enable --now pigpiod

if ! python3 -c "import cv2" 2>/dev/null; then
  log "OpenCV 설치"
  sudo apt-get install -y python3-opencv
fi

# --- MediaMTX -------------------------------------------------------------
case "$(uname -m)" in
  aarch64) MTX_ARCH=linux_arm64 ;;
  armv7l)  MTX_ARCH=linux_armv7   ;;
  x86_64)  MTX_ARCH=linux_amd64   ;;
  *) echo "지원하지 않는 아키텍처: $(uname -m)" >&2; exit 1 ;;
esac

if [[ -x "${MEDIAMTX_DIR}/mediamtx" ]]; then
  log "MediaMTX 이미 설치됨: ${MEDIAMTX_DIR}/mediamtx"
else
  log "MediaMTX 최신 릴리스 조회"
  TAG="$(curl -fsSL https://api.github.com/repos/bluenviron/mediamtx/releases/latest \
         | grep -m1 '"tag_name"' | cut -d'"' -f4)"
  [[ -n "$TAG" ]] || { echo "릴리스 태그를 가져오지 못했습니다." >&2; exit 1; }
  URL="https://github.com/bluenviron/mediamtx/releases/download/${TAG}/mediamtx_${TAG}_${MTX_ARCH}.tar.gz"

  log "MediaMTX ${TAG} (${MTX_ARCH}) 다운로드"
  TMP="$(mktemp -d)"
  trap 'rm -rf "$TMP"' EXIT
  curl -fsSL "$URL" -o "$TMP/mediamtx.tar.gz"
  tar -xzf "$TMP/mediamtx.tar.gz" -C "$TMP"
  sudo mkdir -p "$MEDIAMTX_DIR"
  sudo install -m 0755 "$TMP/mediamtx" "$MEDIAMTX_DIR/mediamtx"
fi

log "프로젝트 mediamtx.yml 연결"
sudo ln -sfn "${ROOT}/mediamtx.yml" "${MEDIAMTX_DIR}/mediamtx.yml"

cat <<MSG

설치 완료.

  1) 검출 모델 내려받기:   ./scripts/download_model.sh
  2) 서보 방향/범위 확인:  python3 scripts/calibrate_servo.py
  3) 실행:                 ./scripts/run.sh

  systemd 등록은 systemd/README 참고.
MSG

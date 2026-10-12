#!/usr/bin/env bash
# 의존성 설치: 빌드 도구·라이브러리, ffmpeg, MediaMTX
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MEDIAMTX_DIR="${MEDIAMTX_DIR:-/opt/mediamtx}"

log() { printf '\033[1;32m==>\033[0m %s\n' "$*"; }

log "APT 패키지 설치"
sudo apt-get update
# pigpio 는 motor.backend=servo 와 tools/servo_test 용이다. 헤더가 있어야 빌드된다.
# 데몬(pigpiod)은 켜지 않는다 — 기본 백엔드 kservo 는 커널 모듈이 같은 핀을 쓴다.
sudo apt-get install -y \
  build-essential pkg-config libopencv-dev libcamera-dev libcamera-apps \
  ffmpeg pigpio curl ca-certificates

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

# make run 과 scripts/*.sh 는 저장소 안의 bin/mediamtx 를 쓴다.
mkdir -p "${ROOT}/bin"
ln -sfn "${MEDIAMTX_DIR}/mediamtx" "${ROOT}/bin/mediamtx"

cat <<MSG

설치 완료.

  1) 검출 모델 내려받기:   ./scripts/download_model.sh
  2) 빌드:                 make
  3) 커널 서보 드라이버:   README 의 "커널 서보 드라이버" 절
  4) 실행:                 make run

  부팅 시 자동 실행:       ./systemd/install-services.sh
MSG

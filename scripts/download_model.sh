#!/usr/bin/env bash
# MobileNet-SSD (Caffe) 모델 내려받기. VOC 20클래스, person = 15.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ROOT}/models"
mkdir -p "$DEST"

# ★ prototxt(설명서)와 caffemodel(가중치)은 반드시 같은 출처에서 받을 것.
#
# 섞어 받으면 조용히 실패한다. chuanqi305 의 deploy.prototxt 는 BatchNorm 층이
# 35개 있는 학습용 구조이고, 배포용 caffemodel 은 그 BatchNorm 이 이미 Conv 에
# 합쳐져 있다. 둘을 짝지으면 로드는 되는 듯하다가 추론에서 터진다:
#   batch_norm_layer.cpp:47: (-215) blobs.size() >= 2
BASE="https://github.com/djmv/MobilNet_SSD_opencv/raw/master"
PROTO_URL="${BASE}/MobileNetSSD_deploy.prototxt"
WEIGHTS_URL="${BASE}/MobileNetSSD_deploy.caffemodel"

fetch() {  # fetch <출력경로> <url>
  local out="$1" url="$2"
  if [[ -s "$out" ]]; then
    echo "이미 존재: $out"
    return 0
  fi
  echo "다운로드: $url"
  curl -fL --retry 3 --progress-bar -o "$out.tmp" "$url"
  mv "$out.tmp" "$out"
}

fetch "$DEST/MobileNetSSD_deploy.prototxt" "$PROTO_URL"
fetch "$DEST/MobileNetSSD_deploy.caffemodel" "$WEIGHTS_URL"

# 검증은 반드시 앱과 같은 OpenCV 로 해야 한다.
# (예전엔 파이썬 cv2 로 검사했는데, 그건 pip 로 깔린 4.13 이라 4.5.1 에서 터지는
#  위 문제를 통과시켜 버렸다. --check-model 은 앱 자신이 로드 + 추론 1회를 돌린다.)
BIN="${ROOT}/build/camtracker"
if [[ -x "$BIN" ]]; then
  echo "검증 중..."
  "$BIN" --check-model
else
  echo "경고: ${BIN} 가 없어 검증을 건너뜁니다. 'make' 후 '$BIN --check-model' 로 확인하세요." >&2
fi

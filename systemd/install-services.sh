#!/usr/bin/env bash
# systemd 서비스 등록 (부팅 시 자동 실행)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USER_NAME="${SUDO_USER:-$USER}"

sudo install -m 0644 "$ROOT/systemd/mediamtx.service" /etc/systemd/system/mediamtx.service
sed -e "s|__ROOT__|$ROOT|g" -e "s|__USER__|$USER_NAME|g" \
    "$ROOT/systemd/camtracker.service" | sudo tee /etc/systemd/system/camtracker.service >/dev/null

sudo systemctl daemon-reload
sudo systemctl enable --now mediamtx.service camtracker.service
systemctl --no-pager status mediamtx.service camtracker.service || true

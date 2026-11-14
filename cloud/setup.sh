#!/bin/bash
# 1. Prepare a fresh Ubuntu 24.04 server: system libraries, Python 3.11 via uv, the venv at
#    ~/motobat/venv (same path as here). Run once per server. ~5 min.
set -e; source "$(dirname "$0")/env.sh"
$SSH 'set -e
export DEBIAN_FRONTEND=noninteractive
apt-get update -q && apt-get install -y -q rsync curl libgl1 libgomp1 libegl1 libglib2.0-0 libx11-6 >/dev/null
command -v ~/.local/bin/uv >/dev/null || curl -LsSf https://astral.sh/uv/install.sh | sh
~/.local/bin/uv python install 3.11.15
~/.local/bin/uv venv --python 3.11.15 ~/motobat/venv
mkdir -p ~/projects/MotoBat
nproc; free -g | head -2; df -h / | tail -1'
echo "server ready; next: bash cloud/push.sh"

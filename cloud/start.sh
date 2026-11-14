#!/bin/bash
# 4. Start the run on the server, detached (it keeps going if this laptop sleeps).
#    Usage: bash cloud/start.sh [WORKERS]   (default 14 for 64 vCPU: ~4.5 vCPU per case)
set -e; source "$(dirname "$0")/env.sh"
W=${1:-14}
$SSH "cd $REMOTE && setsid nohup bash cloud/run_remote.sh $W > data/cloud_run.out 2>&1 < /dev/null & sleep 2; tail -3 data/PROGRESS.md"
echo "running; watch with: bash cloud/status.sh   (or: bash cloud/status.sh --wait)"

#!/bin/bash
# 5. Copy the results home, into a NEW folder (never over data/dataset), and check every case came.
set -e; source "$(dirname "$0")/env.sh"; cd "$REPO"
DST=data/dataset_cloud_$(date +%Y%m%d_%H%M)
mkdir -p $DST
rsync -az --exclude _spill root@$HOST:$REMOTE/data/dataset/ $DST/
rsync -az root@$HOST:$REMOTE/data/PROGRESS.md root@$HOST:$REMOTE/data/PROVENANCE.txt root@$HOST:$REMOTE/data/tripwire.out $DST/ 2>/dev/null || true
n=$(ls $DST/*/*/*/*.score.json | wc -l)
echo "pulled to $DST: $n score files (a complete run has 1550 = 775 cases x 2 setups)"
echo $DST > ~/.config/motobat/last_pull

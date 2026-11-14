#!/bin/bash
# 2. Copy the working tree (not git: the run needs uncommitted work and gitignored data/), the
#    inputs the run reads from data/, and install the pinned packages. Records what was sent.
set -e; source "$(dirname "$0")/env.sh"; cd "$REPO"
{ echo "pushed $(date -Is) from $(hostname)"; echo "HEAD $(git rev-parse HEAD)"; git status --short; } > data/PROVENANCE.txt
rsync -az --info=stats1 --exclude venv/ --exclude .venv/ --exclude build/ --exclude install/ --exclude log/ \
  --exclude src/_vendor/ --exclude __pycache__/ --exclude .pytest_cache/ --exclude /data/ --exclude .git/ \
  ./ root@$HOST:$REMOTE/
$SSH "mkdir -p $REMOTE/data/coverage $REMOTE/data/probes"
rsync -az data/coverage/*.json data/coverage/*.npz root@$HOST:$REMOTE/data/coverage/
rsync -az data/arm_memmax data/tripwire.py data/pause.sh data/PROVENANCE.txt root@$HOST:$REMOTE/data/
rsync -az data/probes/run_case.py root@$HOST:$REMOTE/data/probes/
$SSH "mkdir -p $REMOTE/data/known_cases_runs"
if [ -f data/known_cases_runs/LAST_PASS.json ]; then rsync -az data/known_cases_runs/LAST_PASS.json root@$HOST:$REMOTE/data/known_cases_runs/
else echo "WARNING: no known-cases pass recorded here; the server will refuse to start (python defectgen/known_cases/run.py)"; fi
$SSH "cd $REMOTE/defectgen && ~/.local/bin/uv pip install -q --python ~/motobat/venv/bin/python -r requirements-lock.txt \
  && ~/.local/bin/uv pip install -q --python ~/motobat/venv/bin/python --no-deps -e ."
echo "pushed; next: bash cloud/verify.sh"

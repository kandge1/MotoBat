#!/bin/bash
# Resume what tools/pause.sh stopped:   bash tools/resume.sh  [--dry-run]
# Relaunches each command in data/PAUSED from its own folder, detached, so the terminal can be
# closed. A chain that detaches itself (it handles `--detached`) is started the way it is started by
# hand; anything else under setsid + nohup, output to data/resumed_<time>.out.
# Refuses if a run is already going: two runs on one dataset share scratch files (SIGBUS, 11 Nov).
cd "$(dirname "$0")/.." || exit 1
STATE=data/PAUSED
DRY=0; [ "$1" = "--dry-run" ] && DRY=1
[ -f $STATE ] || { echo "nothing paused ($STATE not found)"; exit 1; }

running=$(ps -eo pid=,args= | grep -E "\.sh --detached|cloud/run_remote\.sh|defectgen\.dataset|known_cases/run\.py|analysis\.sensitivity" \
          | grep -v grep | grep -v " $$ ")
if [ -n "$running" ]; then echo "a run is already going, not starting another:"; echo "$running"; exit 1; fi

stamp=$(date +%Y%m%d_%H%M)
grep -v '^#' $STATE | while IFS=$'\t' read -r dir cmd; do
  [ -z "$cmd" ] && continue
  script=$(eval "set -- $cmd"; for a; do case "$a" in *.sh) echo "$a"; break;; esac; done)
  if [ -n "$script" ] && (cd "$dir" && grep -q -- '--detached' "$script"); then how="self-detaching"; else how="setsid nohup"; fi
  echo "resume ($how) in $dir:  $cmd"
  [ $DRY = 1 ] && continue
  if [ "$how" = "self-detaching" ]; then
    (cd "$dir" && eval "$cmd")
  else
    (cd "$dir" && eval "setsid nohup $cmd > data/resumed_$stamp.out 2>&1 < /dev/null &")
  fi
done
[ $DRY = 1 ] && { echo "(dry run: nothing started)"; exit 0; }
mv $STATE "$STATE.resumed_$stamp"
echo "resumed."

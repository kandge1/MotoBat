#!/bin/bash
# Runs ON the server (started by cloud/start.sh). Every tier split across W workers with
# `--shard i/W`, arm first (clean, pod, grr), then fixed. Each worker memory-capped as on the
# laptop. Scan scratch in RAM (/dev/shm). data/tripwire.py pauses everything on systematic
# false alarms. Resumable: run it again and finished cases are skipped.
cd ~/projects/MotoBat || exit 1
source ~/motobat/venv/bin/activate
# The same code that passed the known cases on the laptop (defectgen/known_cases): same files, same hash.
KC=$(python defectgen/known_cases/run.py --check 2>&1) || { echo "- STOPPED: $KC" >> data/PROGRESS.md; echo "$KC"; exit 1; }
W=${1:-14}
export DEFECTGEN_THREADS=${THREADS:-4}
P=data/PROGRESS.md
mkdir -p data/dataset /dev/shm/motobat_spill
[ -e data/dataset/_spill ] || ln -s /dev/shm/motobat_spill data/dataset/_spill
rm -f data/CLOUD_DONE
python3 data/tripwire.py >> data/tripwire.out 2>&1 &
TRIP=$!
echo "- $(date '+%a %H:%M'): cloud run started, $W workers, $(nproc) vCPU" >> $P
for s in arm fixed; do
  m=10G; [ $s = arm ] && m=$(cat data/arm_memmax)
  for t in clean pod grr; do
    for try in 1 2 3; do
      pids=()
      for i in $(seq 0 $((W - 1))); do
        systemd-run --scope -q -p MemoryMax=$m -p MemorySwapMax=0 \
          python -m defectgen.dataset --tier $t --setups $s --shard $i/$W >> data/dataset/run_${s}_${t}_$i.log 2>&1 &
        pids+=($!)
      done
      fail=0; for p in "${pids[@]}"; do wait $p || fail=1; done
      [ $fail = 0 ] && break
      echo "- $(date '+%a %H:%M'): $s $t: a worker failed (try $try); rerunning, finished cases are kept" >> $P
    done
    [ $fail = 0 ] || { echo "- STOPPED: $s $t failed 3 times, see data/dataset/run_${s}_${t}_*.log" >> $P; kill $TRIP; exit 1; }
    echo "- $(date '+%a %H:%M'): cloud $s $t done" >> $P
  done
done
kill $TRIP 2>/dev/null
echo "- $(date '+%a %H:%M'): cloud run finished" >> $P
touch data/CLOUD_DONE

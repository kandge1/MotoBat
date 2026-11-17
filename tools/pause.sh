#!/bin/bash
# Pause whatever dataset work is running (laptop or Vultr), so it can be resumed later, even after a
# reboot:   bash tools/pause.sh  [--dry-run]
#
# What counts as a run:
#   - a chain: any script started as `<script>.sh --detached` (data/check_f10.sh, chain_v3.sh, ...),
#     or cloud/run_remote.sh;
#   - a job running on its own: defectgen.dataset, known_cases/run.py, analysis.sensitivity.
# Each chain is stopped as a whole process group (the chain, its memory-capped job, the tripwire),
# so it cannot restart the job it is waiting on. Kills by PID / PGID, never by pattern.
#
# Finished cases are kept (every chain and job here skips them). The case in progress is lost and
# redone on resume: at most one case per worker, ~3 min on the arm.
#
# What was stopped is written to data/PAUSED (cwd + exact command); tools/resume.sh relaunches it.
# Run in a session of our own: when the cloud tripwire calls this, the tripwire is in the chain's
# process group, and stopping that group must not stop this script halfway.
if [ "$PAUSE_OWN_SESSION" != 1 ]; then PAUSE_OWN_SESSION=1 exec setsid --wait bash "$0" "$@"; fi
cd "$(dirname "$0")/.." || exit 1
REPO=$(pwd)
DRY=0; [ "$1" = "--dry-run" ] && DRY=1
STATE=data/PAUSED

chains=(); jobs=()
while read -r pid pgid args; do
  [ "$pid" = "$$" ] && continue
  case "$args" in
    "bash "*".sh --detached"*|"bash cloud/run_remote.sh"*) chains+=("$pid:$pgid") ;;   # anchored: not an ssh line that mentions them
    *"python"*"defectgen.dataset"*|*"python"*"known_cases/run.py"*|*"python"*"analysis.sensitivity"*) jobs+=("$pid:$pgid") ;;
  esac
done < <(ps -eo pid=,pgid=,args=)

if [ ${#chains[@]} -eq 0 ] && [ ${#jobs[@]} -eq 0 ]; then echo "nothing running"; exit 0; fi

cmd_of() {   # the process's command line, shell-quoted, without --detached
  local out="" a
  while IFS= read -r -d '' a; do [ "$a" = "--detached" ] || out+="$(printf '%q' "$a") "; done < /proc/$1/cmdline
  echo "${out% }"
}
cwd_of() { readlink /proc/$1/cwd; }

declare -A chain_pg
groups=(); lines=()
for c in "${chains[@]}"; do
  pid=${c%%:*}; pg=${c##*:}
  chain_pg[$pg]=1; groups+=("$pg")
  lines+=("$(cwd_of $pid)	$(cmd_of $pid)")
done
loose=()
for j in "${jobs[@]}"; do   # a job inside a chain's group goes with its chain
  pid=${j%%:*}; pg=${j##*:}
  [ -n "${chain_pg[$pg]}" ] && continue
  loose+=("$pid"); lines+=("$(cwd_of $pid)	$(cmd_of $pid)")
done

echo "will stop:"
for l in "${lines[@]}"; do echo "  ${l#*	}   (in ${l%%	*})"; done
for g in "${groups[@]}"; do echo "  process group $g:"; ps -o pid=,args= -g "$g" | sed 's/^/      /'; done
for p in "${loose[@]}"; do echo "  pid $p"; done
[ $DRY = 1 ] && { echo "(dry run: nothing stopped)"; exit 0; }

mkdir -p data
{ echo "# paused $(date '+%a %d %b %H:%M') by tools/pause.sh; resume: bash tools/resume.sh"
  printf '%s\n' "${lines[@]}"; } > $STATE

for g in "${groups[@]}"; do kill -TERM -- "-$g" 2>/dev/null; done
for p in "${loose[@]}"; do kill -TERM "$p" 2>/dev/null; done
for i in $(seq 20); do   # up to 20 s to exit, then force
  alive=0
  for g in "${groups[@]}"; do ps -g "$g" >/dev/null 2>&1 && alive=1; done
  for p in "${loose[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
  [ $alive = 0 ] && break; sleep 1
done
for g in "${groups[@]}"; do ps -g "$g" >/dev/null 2>&1 && kill -KILL -- "-$g" 2>/dev/null; done
for p in "${loose[@]}"; do kill -0 "$p" 2>/dev/null && kill -KILL "$p" 2>/dev/null; done

for f in data/PROGRESS.md data/dataset_*/PROGRESS.md; do
  [ -f "$f" ] && [ "$(find "$f" -mmin -1440)" ] && echo "- $(date '+%a %H:%M'): PAUSED by tools/pause.sh (resume: bash tools/resume.sh)" >> "$f"
done
echo "paused. What to resume is in $REPO/$STATE"

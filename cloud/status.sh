#!/bin/bash
# Progress on the server. --wait: check every 5 min until done or stopped, then ring the bell.
source "$(dirname "$0")/env.sh"
show() {
  $SSH "cd $REMOTE && for s in arm fixed; do for t in clean pod grr; do
      n=\$(ls data/dataset/\$t/*/*/\$s.score.json 2>/dev/null | wc -l); printf '%-6s %-6s %4s\n' \$s \$t \$n; done; done
    echo 'targets: clean 30, pod 565, grr 180'; tail -4 data/PROGRESS.md; uptime; free -g | sed -n 2p
    [ -e data/CLOUD_DONE ] && echo CLOUD_DONE; grep -q 'TRIPWIRE\|STOPPED' data/PROGRESS.md && echo ATTENTION || true"
}
if [ "$1" = --wait ]; then
  while :; do out=$(show); clear; echo "$out"; echo "$out" | grep -q "CLOUD_DONE\|ATTENTION" && { printf '\a'; break; }; sleep 300; done
else show; fi

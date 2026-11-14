#!/bin/bash
# 3. Before paying for hours: the tests, then two known cases, which must come out as they do on
#    the laptop (outcome, false alarms, and the measurement to ~1e-9).  ~10 min.
set -e; source "$(dirname "$0")/env.sh"
$SSH "cd $REMOTE && source ~/motobat/venv/bin/activate && cd defectgen && python -m pytest -q -x tests 2>&1 | tail -2"
$SSH "cd $REMOTE && source ~/motobat/venv/bin/activate && rm -rf data/probes/verify && \
  python data/probes/run_case.py data/probes/verify arm pod cell_shift L0_0.1_s0 </dev/null 2>/dev/null | grep '^arm' && \
  python data/probes/run_case.py data/probes/verify fixed pod cell_shift L0_0.1_s0 </dev/null 2>/dev/null | grep '^fixed'"
echo "expected (laptop, v3 detector):"
echo "  arm   cell_shift L0_0.1_s0: pass  meas 0.1025  fa 0 other 0"
echo "  fixed cell_shift L0_0.1_s0: pass  meas 0.1448  fa 0 other 0"
echo "If they match: bash cloud/start.sh 14"

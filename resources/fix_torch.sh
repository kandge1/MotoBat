#!/usr/bin/env bash
# =============================================================================
#  MotoBat — PyTorch repair
#
#  WHAT BROKE
#  The installer ran, in order:
#      pip install torch torchvision torchaudio --index-url .../cu124   -> 2.6.0+cu124
#      pip install "stable-baselines3[extra]" gymnasium                 -> (no index-url)
#
#  The second command had no --index-url, so pip resolved torch from PyPI,
#  found 2.13.0, and upgraded over the CUDA build. torchvision 0.21.0 and
#  torchaudio 2.6.0 both pin torch==2.6.0, so they are now broken, and the
#  new torch pulled cuda-13 wheels alongside the cu124 ones.
#
#  This is my bug, not yours. The fix is to reinstall all four together from
#  the CUDA index in a single resolve.
# =============================================================================
set -Eeuo pipefail

VENV="${HOME}/motobat/venv"
PIP="${VENV}/bin/pip"
PY="${VENV}/bin/python"
INDEX="https://download.pytorch.org/whl/cu124"

[[ -x "${PIP}" ]] || { echo "No venv at ${VENV}"; exit 1; }

echo ":: current state"
"${PIP}" list 2>/dev/null | grep -iE "^(torch|torchvision|torchaudio|numpy|open3d) " || true

echo ""
echo ":: removing conflicting torch stack and stray cuda-13 wheels"
"${PIP}" uninstall -y torch torchvision torchaudio >/dev/null 2>&1 || true
# SB3's PyPI torch dragged in a parallel CUDA 13 runtime. Leaving it wastes
# ~4 GB and can shadow the cu124 libraries at import time.
"${PIP}" uninstall -y \
  nvidia-cublas-cu13 nvidia-cuda-cupti-cu13 nvidia-cuda-nvrtc-cu13 \
  nvidia-cuda-runtime-cu13 nvidia-cudnn-cu13 nvidia-cufft-cu13 \
  nvidia-cufile-cu13 nvidia-curand-cu13 nvidia-cusolver-cu13 \
  nvidia-cusparse-cu13 nvidia-cusparselt-cu13 nvidia-nccl-cu13 \
  nvidia-nvjitlink-cu13 nvidia-nvshmem-cu13 nvidia-nvtx-cu13 \
  cuda-toolkit cuda-bindings cuda-pathfinder triton \
  >/dev/null 2>&1 || true

echo ":: reinstalling the CUDA stack in one resolve (pinned)"
"${PIP}" install --index-url "${INDEX}" \
  "torch==2.6.0+cu124" "torchvision==0.21.0+cu124" "torchaudio==2.6.0+cu124"

echo ":: reinstalling stable-baselines3 WITHOUT letting it touch torch"
# --no-deps is the key: SB3's only heavy dep is torch, which we just pinned.
"${PIP}" install --no-deps "stable-baselines3==2.9.0"
"${PIP}" install "gymnasium>=1.0" cloudpickle pandas matplotlib

echo ":: pinning protobuf to satisfy grpcio-tools"
"${PIP}" install "protobuf>=7.35.1,<8.0.0" || \
  echo "  ! protobuf pin failed — harmless unless you generate gRPC stubs"

echo ""
echo ":: verifying"
"${PY}" - <<'PYCHK'
import sys
ok = True
try:
    import torch, torchvision
    print(f"  torch        {torch.__version__}")
    print(f"  torchvision  {torchvision.__version__}")
    print(f"  CUDA avail   {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"  GPU          {torch.cuda.get_device_name(0)}")
    else:
        print("  ! CUDA not available — check driver with nvidia-smi")
        ok = False
except Exception as e:
    print(f"  x torch/torchvision: {e}"); ok = False
try:
    import open3d, numpy
    print(f"  open3d       {open3d.__version__}")
    print(f"  numpy        {numpy.__version__}")
except Exception as e:
    print(f"  x open3d/numpy: {e}"); ok = False
sys.exit(0 if ok else 1)
PYCHK

echo ""
echo "done."

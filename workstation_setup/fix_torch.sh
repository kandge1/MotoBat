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

VENV="${MOTOBAT_VENV:-${HOME}/motobat/venv}"
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
# ~4 GB and shadows the cu124 libraries at import time.
#
# DO NOT guess these names. An earlier version of this script uninstalled
# "nvidia-cublas-cu13", "nvidia-cudnn-cu13" and friends -- none of which exist.
# NVIDIA publishes the CUDA 13 wheels UNSUFFIXED ("nvidia-cublas 13.1.1.3"),
# keeping the -cu12 suffix only on the CUDA 12 line. So the strays survived,
# the uninstall silently no-op'd, and torch imported against a half-removed
# runtime. Select by shape instead: every nvidia-* wheel that is NOT -cu12.
mapfile -t STRAYS < <(
  "${PIP}" list --format=freeze 2>/dev/null \
    | cut -d= -f1 \
    | grep -E '^nvidia-' \
    | grep -vE -- '-cu12$'
)
if (( ${#STRAYS[@]} )); then
  echo "   found ${#STRAYS[@]} CUDA-13 wheel(s): ${STRAYS[*]}"
  "${PIP}" uninstall -y "${STRAYS[@]}" >/dev/null 2>&1 || true
else
  echo "   none present"
fi
"${PIP}" uninstall -y cuda-toolkit cuda-bindings cuda-pathfinder triton \
  >/dev/null 2>&1 || true

# --force-reinstall is deliberate and not paranoia. Uninstalling the unsuffixed
# wheels can delete files that the -cu12 packages still claim in their metadata,
# leaving pip convinced nvidia-cudnn-cu12 is installed while libcudnn.so.9 is
# gone from disk. Forcing the resolve restores every dependency's files.
echo ":: reinstalling the CUDA stack in one resolve (pinned, forced)"
"${PIP}" install --force-reinstall --index-url "${INDEX}" \
  "torch==2.6.0+cu124" "torchvision==0.21.0+cu124" "torchaudio==2.6.0+cu124"

# --force-reinstall above drags numpy 2.x in as a torch dependency. That must
# not stand: Open3D 0.19 wheels and several Jazzy Python packages link the
# numpy 1.x ABI, and the breakage is an ugly one to diagnose. See §1.3.
echo ":: restoring the numpy 1.x pin (torch pulls 2.x; Open3D needs 1.x)"
"${PIP}" install "numpy==1.26.4"

echo ":: reinstalling stable-baselines3 WITHOUT letting it touch torch"
# --no-deps is the key: SB3's only heavy dep is torch, which we just pinned.
# SB3 2.9.0 requires torch>=2.8 and will NOT work against the 2.6.0+cu124 we
# pin for CUDA 12.4 / Isaac Sim. 2.5.0 is the last line that accepts torch 2.6.
"${PIP}" install --no-deps "stable-baselines3==2.5.0"
# SB3 2.5.0 caps gymnasium below 1.1; pinning it here avoids a pip check warning.
"${PIP}" install "gymnasium>=0.29.1,<1.1.0" cloudpickle pandas matplotlib typeguard

echo ":: pinning protobuf to satisfy grpcio-tools"
# This leaves one benign `pip check` complaint: databricks-sdk (pulled in by
# mlflow) wants protobuf <7. We do not use mlflow's databricks backend, and
# grpcio-tools needs 7.x for the inspector-to-correction stubs. Ignore it.
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
        # Actually exercise the runtime. is_available() returns True against a
        # half-installed cuDNN/NCCL; a real matmul is what catches that.
        x = torch.randn(512, 512, device="cuda")
        torch.cuda.synchronize()
        print(f"  cuda matmul  {tuple((x @ x).shape)} ok")
    else:
        print("  ! CUDA not available — check driver with nvidia-smi")
        ok = False
except Exception as e:
    print(f"  x torch/torchvision: {e}"); ok = False
try:
    import open3d, numpy
    print(f"  open3d       {open3d.__version__}")
    print(f"  numpy        {numpy.__version__}")
    # Guard the pin explicitly. torch's resolve pulls numpy 2.x, which breaks
    # the Open3D ABI in ways that surface far from here.
    if not numpy.__version__.startswith("1."):
        print(f"  x numpy {numpy.__version__} — must be 1.x for Open3D"); ok = False
    # Exercise Open3D against numpy rather than trusting the import.
    import numpy as np
    pc = open3d.geometry.PointCloud()
    pc.points = open3d.utility.Vector3dVector(np.random.rand(100, 3))
    assert np.asarray(pc.points).shape == (100, 3)
    print("  open3d/numpy interop ok")
except Exception as e:
    print(f"  x open3d/numpy: {e}"); ok = False
sys.exit(0 if ok else 1)
PYCHK

echo ""
echo "done."

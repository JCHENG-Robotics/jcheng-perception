#!/usr/bin/env bash
# Local pre-push checks: Python syntax, core tests, fusion simulation.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

echo "== compile =="
python3 -m compileall -q ros2_ws/src scripts tests
echo "ok"

echo "== unit tests =="
python3 -m pytest tests -q

echo "== detector mock smoke =="
python3 scripts/check_gpu_backend.py --backend mock --json > /dev/null
echo "ok"

echo "== simulation =="
python3 scripts/simulate_pipeline.py

echo "== no model weights in the tree =="
if find . -path ./.git -prune -o \
    \( -name '*.pt' -o -name '*.onnx' -o -name '*.engine' \
       -o -name '*_openvino_model' \) -print | grep -q .; then
  echo "model weights found; remove them before committing" >&2
  exit 1
fi
echo "ok"

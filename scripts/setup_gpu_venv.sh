#!/usr/bin/env bash
# Create the optional GPU-node Python environment.
#
# The ROS packages stay installed in the system/ROS Python environment. This
# venv uses --system-site-packages and only adds the detector runtime; launch
# the node with python_executable pointing at its interpreter.
#
# Default: ONNX Runtime CPU (small, no torch). For a CUDA/Ultralytics host:
#   RUNTIME=ultralytics ./scripts/setup_gpu_venv.sh
set -euo pipefail

VENV="${JCHENG_GPU_VENV:-$HOME/.venvs/jcheng-perception-gpu}"
RUNTIME="${RUNTIME:-onnxruntime}"
INDEX="${JCHENG_PIP_INDEX:-https://pypi.org/simple}"
UV="${UV:-uv}"

if ! command -v "${UV}" >/dev/null 2>&1; then
  echo "uv is required (https://github.com/astral-sh/uv)" >&2
  exit 1
fi

"${UV}" venv --python /usr/bin/python3 --system-site-packages "${VENV}"

case "${RUNTIME}" in
  onnxruntime)
    "${UV}" pip install --python "${VENV}/bin/python" \
      --index-url "${INDEX}" onnxruntime "protobuf>=4.25.8" flatbuffers
    ;;
  openvino)
    "${UV}" pip install --python "${VENV}/bin/python" \
      --index-url "${INDEX}" openvino openvino-telemetry "numpy<2"
    ;;
  ultralytics)
    "${UV}" pip install --python "${VENV}/bin/python" \
      --index-url "${INDEX}" ultralytics openvino "numpy<2"
    ;;
  ultralytics-cpu)
    # Small CPU-only install: PyTorch CPU wheels plus the Ultralytics
    # dependencies, without letting the resolver pull a CUDA torch build.
    "${UV}" pip install --python "${VENV}/bin/python" \
      --index-url https://download.pytorch.org/whl/cpu torch torchvision
    "${UV}" pip install --python "${VENV}/bin/python" \
      --index-url "${INDEX}" "numpy<2" matplotlib opencv-python pillow \
      PyYAML requests scipy tqdm psutil py-cpuinfo pandas seaborn \
      ultralytics-thop polars
    "${UV}" pip install --python "${VENV}/bin/python" \
      --index-url "${INDEX}" --no-deps ultralytics
    ;;
  *)
    echo "unknown RUNTIME=${RUNTIME} (expected onnxruntime, openvino, ultralytics or ultralytics-cpu)" >&2
    exit 2
    ;;
esac

echo "venv: ${VENV}"
echo "python: ${VENV}/bin/python"
echo "Run the GPU node with:"
echo "  ros2 launch jcheng_perception_gpu laptop_gpu.launch.py \\"
echo "    python_executable:=${VENV}/bin/python"

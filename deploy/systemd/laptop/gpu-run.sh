#!/usr/bin/env bash
# Run the laptop GPU detector as a user service.
#
# The Pi ingresses publish JPEG uplinks on domain 0; this node runs YOLO on the
# RTX 3060 and returns detections with the original capture stamp.  It is
# read-only with respect to the robot (no camera device, no motion).
set -euo pipefail

WS="${JCHENG_GPU_WS:-$HOME/jcheng/jcheng-perception/ros2_ws}"
CONFIG="${JCHENG_GPU_CONFIG:-$HOME/.config/jcheng-perception/gpu_laptop.yaml}"
PYTHON="${JCHENG_GPU_PYTHON:-$HOME/.venvs/jcheng-perception-gpu/bin/python}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/.config/jcheng-perception/cyclonedds-laptop.xml}"
export ROS2CLI_NO_DAEMON=1

[[ -f "$WS/install/setup.bash" ]] || {
    echo "missing $WS/install/setup.bash" >&2
    exit 1
}
[[ -x "$PYTHON" ]] || {
    echo "missing executable $PYTHON (venv with torch/ultralytics)" >&2
    exit 1
}
[[ -f "$CONFIG" ]] || {
    echo "missing GPU config $CONFIG" >&2
    exit 1
}
[[ -f "${CYCLONEDDS_URI#file://}" ]] || {
    echo "missing CycloneDDS config ${CYCLONEDDS_URI#file://}" >&2
    exit 1
}

# ROS setup scripts are not -u safe (AMENT_TRACE_SETUP_FILES).
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

exec ros2 launch jcheng_perception_gpu laptop_gpu.launch.py \
    gpu_config:="$CONFIG" \
    python_executable:="$PYTHON"

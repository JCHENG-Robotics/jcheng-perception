#!/usr/bin/env bash
# Run the GPU reader-wedge watchdog as a user service.
#
# It is a small ROS node on domain 0: it subscribes to /perception/gpu_status
# and restarts jcheng-perception-gpu.service only when one camera's
# frames_received is frozen while another advances (or status is missing).
set -euo pipefail

WS="${JCHENG_GPU_WS:-$HOME/jcheng/jcheng-perception/ros2_ws}"
SCRIPT="${JCHENG_GPU_WATCHDOG_SCRIPT:-$HOME/.local/lib/jcheng-perception/gpu-watchdog.py}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/.config/jcheng-perception/cyclonedds-laptop.xml}"
export ROS2CLI_NO_DAEMON=1

[[ -f "$WS/install/setup.bash" ]] || {
    echo "missing $WS/install/setup.bash" >&2
    exit 1
}
[[ -f "$SCRIPT" ]] || {
    echo "missing $SCRIPT" >&2
    exit 1
}

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

exec python3 "$SCRIPT"

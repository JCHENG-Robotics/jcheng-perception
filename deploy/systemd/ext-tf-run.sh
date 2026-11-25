#!/usr/bin/env bash
# Start the external-camera static TF publisher inside the vendor container.
#
# Called by jcheng-perception-ext-tf.service.  It publishes base_link ->
# camera_link from the static fallback in frames.yaml; the RealSense driver
# already publishes camera_link -> camera_color_optical_frame, so the external
# optical frame becomes reachable.  Read-only.
set -euo pipefail

CONTAINER="${JCHENG_PERCEPTION_CONTAINER:-ArmPiUltra}"
CONTAINER_USER="${JCHENG_PERCEPTION_CONTAINER_USER:-ubuntu}"
WS="${JCHENG_PERCEPTION_WS:-/home/ubuntu/jcheng-perception/ros2_ws}"
WAIT_SECONDS="${JCHENG_PERCEPTION_CONTAINER_WAIT_S:-60}"
# Set by ExecStop before it kills the node; see ingress-run.sh.
MARKER="/tmp/jcheng-perception-ext-tf.stopping"
rm -f "$MARKER"
# See ingress-run.sh: a deferred TERM trap turns systemd's stop into exit 0.
STOP_REQUESTED=0
trap 'STOP_REQUESTED=1' TERM INT

CFG_DIR="${WS%/}/install/jcheng_perception_edge/share/jcheng_perception_edge/config"
EXT_TF_CONFIG="${JCHENG_PERCEPTION_EXT_TF_CONFIG:-$CFG_DIR/ext_tf.yaml}"
FRAMES_CONFIG="${JCHENG_PERCEPTION_FRAMES_CONFIG:-$CFG_DIR/frames.yaml}"

deadline=$((SECONDS + WAIT_SECONDS))
until docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; do
    if (( SECONDS >= deadline )); then
        echo "ERROR: container ${CONTAINER} is not running after ${WAIT_SECONDS}s" >&2
        exit 1
    fi
    sleep 2
done

rc=0
docker exec -u "$CONTAINER_USER" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
    -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
    -e "CYCLONEDDS_URI=${CYCLONEDDS_URI:-file:///home/ubuntu/cyclonedds.xml}" \
    "$CONTAINER" /bin/bash -lc "
        set -eo pipefail
        source /opt/ros/humble/setup.bash
        source '${WS}/install/setup.bash'
        exec ros2 run jcheng_perception_edge ext_tf --ros-args \
            -r __node:=ext_tf \
            -p frames_config:='${FRAMES_CONFIG}' \
            -p config_file:='${EXT_TF_CONFIG}'
    " || rc=$?

if [[ "$STOP_REQUESTED" -eq 1 || -f "$MARKER" ]]; then
    rm -f "$MARKER"
    echo "stop requested: container exit ${rc} treated as a normal stop" >&2
    exit 0
fi
exit "$rc"

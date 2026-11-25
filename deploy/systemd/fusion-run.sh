#!/usr/bin/env bash
# Start the perception fusion process inside the vendor container.
#
# Called by jcheng-perception-fusion.service.  It subscribes to the two small
# observation topics and publishes /perception/targets and /perception/status;
# it does not subscribe to any camera stream, so a wedged ingress cannot take
# it down.
set -euo pipefail

CONTAINER="${JCHENG_PERCEPTION_CONTAINER:-ArmPiUltra}"
CONTAINER_USER="${JCHENG_PERCEPTION_CONTAINER_USER:-ubuntu}"
WS="${JCHENG_PERCEPTION_WS:-/home/ubuntu/jcheng-perception/ros2_ws}"
WAIT_SECONDS="${JCHENG_PERCEPTION_CONTAINER_WAIT_S:-60}"
# Set by ExecStop before it kills the node; see ingress-run.sh.
MARKER="/tmp/jcheng-perception-fusion.stopping"
rm -f "$MARKER"
# See ingress-run.sh: a deferred TERM trap turns systemd's stop into exit 0.
STOP_REQUESTED=0
trap 'STOP_REQUESTED=1' TERM INT

CFG_DIR="${WS%/}/install/jcheng_perception_edge/share/jcheng_perception_edge/config"
CAMERAS_CONFIG="${JCHENG_PERCEPTION_CAMERAS_CONFIG:-$CFG_DIR/cameras.yaml}"
FUSION_CONFIG="${JCHENG_PERCEPTION_FUSION_CONFIG:-$CFG_DIR/fusion_dual_hardware.yaml}"

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
        exec ros2 run jcheng_perception_edge perception_fusion --ros-args \
            -r __node:=perception_fusion \
            -p cameras_config:='${CAMERAS_CONFIG}' \
            -p fusion_config:='${FUSION_CONFIG}'
    " || rc=$?

if [[ "$STOP_REQUESTED" -eq 1 || -f "$MARKER" ]]; then
    rm -f "$MARKER"
    echo "stop requested: container exit ${rc} treated as a normal stop" >&2
    exit 0
fi
exit "$rc"

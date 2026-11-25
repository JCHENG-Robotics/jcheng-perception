#!/usr/bin/env bash
# Start one per-camera perception ingress inside the vendor container.
#
# Called by jcheng-perception-ingress@.service as `ingress-run.sh <camera_id>`.
# The camera id is the same key used in config/cameras.yaml (d435i_ext,
# aurora_wrist); it filters the node to one camera and names the process so the
# unit's ExecStop can find it.
#
# The node runs in the foreground so systemd owns its lifetime.  When the 4 s
# ingress watchdog sees a camera reader stall it exits 0 on purpose; systemd
# restarts it (fresh CycloneDDS participant), exactly what launch respawn did.
set -euo pipefail

camera="${1:?usage: ingress-run.sh <camera_id>}"
# `%i` comes from a unit instance name; refuse anything that could escape the
# quoted command below.
if [[ ! "$camera" =~ ^[a-z0-9_]+$ ]]; then
    echo "ERROR: invalid camera id: $camera" >&2
    exit 2
fi

CONTAINER="${JCHENG_PERCEPTION_CONTAINER:-ArmPiUltra}"
CONTAINER_USER="${JCHENG_PERCEPTION_CONTAINER_USER:-ubuntu}"
WS="${JCHENG_PERCEPTION_WS:-/home/ubuntu/jcheng-perception/ros2_ws}"
WAIT_SECONDS="${JCHENG_PERCEPTION_CONTAINER_WAIT_S:-60}"
# ExecStop touches this marker before killing the node; if it is present when
# the node exits, the exit code is a deliberate stop (SIGTERM teardown), not a
# crash.  A stale marker is cleared on start.
MARKER="/tmp/jcheng-perception-ingress-${camera}.stopping"
rm -f "$MARKER"
# systemd may SIGTERM the wrapper while ExecStop kills the node; bash defers
# this trap until the foreground docker exec returns, so the stop still ends
# with a clean exit 0 instead of 143.
STOP_REQUESTED=0
trap 'STOP_REQUESTED=1' TERM INT

CFG_DIR="${WS%/}/install/jcheng_perception_edge/share/jcheng_perception_edge/config"
CAMERAS_CONFIG="${JCHENG_PERCEPTION_CAMERAS_CONFIG:-$CFG_DIR/cameras.yaml}"
FUSION_CONFIG="${JCHENG_PERCEPTION_FUSION_CONFIG:-$CFG_DIR/fusion_dual_hardware.yaml}"
FRAMES_CONFIG="${JCHENG_PERCEPTION_FRAMES_CONFIG:-$CFG_DIR/frames.yaml}"
VIEW_BIAS_CONFIG="${JCHENG_PERCEPTION_VIEW_BIAS_CONFIG:-$CFG_DIR/view_bias.yaml}"

deadline=$((SECONDS + WAIT_SECONDS))
until docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; do
    if (( SECONDS >= deadline )); then
        echo "ERROR: container ${CONTAINER} is not running after ${WAIT_SECONDS}s" >&2
        exit 1
    fi
    sleep 2
done

# The CycloneDDS settings match the camera drivers, the gateway and the GPU
# node.  Mixing RMWs wedged the large image readers on 2026-11-24.
rc=0
docker exec -u "$CONTAINER_USER" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
    -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
    -e "CYCLONEDDS_URI=${CYCLONEDDS_URI:-file:///home/ubuntu/cyclonedds.xml}" \
    "$CONTAINER" /bin/bash -lc "
        set -eo pipefail
        source /opt/ros/humble/setup.bash
        source '${WS}/install/setup.bash'
        exec ros2 run jcheng_perception_edge perception_edge --ros-args \
            -r __node:=perception_ingress_${camera} \
            -p camera_filter:=${camera} \
            -p publish_targets:=false \
            -p publish_status:=false \
            -p ingress_watchdog:=true \
            -p enable_uplink:=true \
            -p cameras_config:='${CAMERAS_CONFIG}' \
            -p fusion_config:='${FUSION_CONFIG}' \
            -p frames_config:='${FRAMES_CONFIG}' \
            -p view_bias_config:='${VIEW_BIAS_CONFIG}'
    " || rc=$?

if [[ "$STOP_REQUESTED" -eq 1 || -f "$MARKER" ]]; then
    rm -f "$MARKER"
    echo "stop requested: container exit ${rc} treated as a normal stop" >&2
    exit 0
fi
exit "$rc"

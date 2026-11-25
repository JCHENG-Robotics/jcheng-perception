#!/usr/bin/env bash
# Install the JCHENG perception split stack as systemd units on the ArmPi host.
#
# Run on the robot as root, from this directory:
#
#   sudo bash install-split-systemd.sh               # install + enable at boot
#   sudo bash install-split-systemd.sh --start       # ... and start now
#   sudo bash install-split-systemd.sh --no-enable   # install only
#   sudo bash install-split-systemd.sh --uninstall   # remove units/scripts
#
# The units run one process each inside the ArmPiUltra container:
#   jcheng-perception-ingress@d435i_ext.service
#   jcheng-perception-ingress@aurora_wrist.service
#   jcheng-perception-fusion.service
#   jcheng-perception-wrist-tf.service   (bus-servo camera TF)
#   jcheng-perception-ext-tf.service     (external static TF)
# started/stopped together through jcheng-perception.target.  The two TF
# publishers are required for metric output; they were hand-started before.
#
# Nothing here moves the arm or opens a camera device on the host: the host
# wrappers only run `docker exec`, and the camera drivers keep device
# ownership.  Do not run split_edge.launch.py while these units are active.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_DIR=/etc/systemd/system
LIB_DIR=/usr/local/lib/jcheng-perception
CONF_DIR=/etc/jcheng-perception
STATE_DIR=/var/lib/jcheng-perception
SYSCTL_FILE=90-cyclonedds-rmem.conf
UNITS=(
    jcheng-perception.target
    jcheng-perception-ingress@.service
    jcheng-perception-fusion.service
    jcheng-perception-wrist-tf.service
    jcheng-perception-ext-tf.service
    jcheng-perception-alert@.service
)
SCRIPTS=(
    ingress-run.sh
    fusion-run.sh
    wrist-tf-run.sh
    ext-tf-run.sh
    alert.sh
)

START=0
ENABLE=1
UNINSTALL=0
for arg in "$@"; do
    case "$arg" in
        --start) START=1 ;;
        --no-enable) ENABLE=0 ;;
        --uninstall) UNINSTALL=1 ;;
        -h|--help) sed -n '2,22p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown argument: $arg" >&2; exit 2 ;;
    esac
done

[[ $EUID -eq 0 ]] || { echo "run as root (sudo bash $0)" >&2; exit 1; }

if [[ "$UNINSTALL" == "1" ]]; then
    echo "== stopping and disabling"
    systemctl disable --now jcheng-perception.target 2>/dev/null || true
    systemctl reset-failed 'jcheng-perception-*' 2>/dev/null || true
    echo "== removing unit files and scripts"
    for unit in "${UNITS[@]}"; do
        rm -f "${UNIT_DIR}/${unit}"
    done
    for script in "${SCRIPTS[@]}"; do
        rm -f "${LIB_DIR}/${script}"
    done
    rmdir "$LIB_DIR" 2>/dev/null || true
    echo "== removing kernel socket-buffer tuning"
    rm -f "/etc/sysctl.d/${SYSCTL_FILE}"
    sysctl --system >/dev/null 2>&1 || true
    systemctl daemon-reload
    echo "kept ${CONF_DIR}/perception.env and ${STATE_DIR}/ (alert history)"
    exit 0
fi

for script in "${SCRIPTS[@]}"; do
    bash -n "${HERE}/${script}" || { echo "syntax error in ${script}" >&2; exit 1; }
done

echo "== installing wrappers and units"
install -d -m 0755 "$LIB_DIR" "$CONF_DIR" "$STATE_DIR"
for script in "${SCRIPTS[@]}"; do
    install -m 0755 "${HERE}/${script}" "${LIB_DIR}/${script}"
done
for unit in "${UNITS[@]}"; do
    install -m 0644 "${HERE}/${unit}" "${UNIT_DIR}/${unit}"
done
echo "== kernel socket buffers"
install -m 0644 "${HERE}/${SYSCTL_FILE}" "/etc/sysctl.d/${SYSCTL_FILE}"
sysctl --system >/dev/null 2>&1 || true
# Keep operator edits: only seed the environment file on first install.
if [[ -f "${CONF_DIR}/perception.env" ]]; then
    echo "keeping existing ${CONF_DIR}/perception.env"
else
    install -m 0644 "${HERE}/perception.env" "${CONF_DIR}/perception.env"
fi

systemctl daemon-reload

if [[ "$ENABLE" == "1" ]]; then
    systemctl enable jcheng-perception.target >/dev/null
    echo "enabled at boot: jcheng-perception.target"
else
    echo "not enabled at boot (--no-enable)"
fi

if [[ "$START" == "1" ]]; then
    systemctl start jcheng-perception.target
    echo "started jcheng-perception.target"
fi

echo "== status"
systemctl list-unit-files --no-pager 'jcheng-perception*' || true
systemctl list-units --all --no-pager 'jcheng-perception*' || true

cat <<'EOF'

Next steps:
  sudo systemctl start jcheng-perception.target     # if not started above
  systemctl status jcheng-perception-ingress@d435i_ext.service
  systemctl status jcheng-perception-ingress@aurora_wrist.service
  systemctl status jcheng-perception-fusion.service
  systemctl status jcheng-perception-wrist-tf.service
  systemctl status jcheng-perception-ext-tf.service
  systemctl show -p NRestarts --value 'jcheng-perception-ingress@*'
  journalctl -u 'jcheng-perception-*' -f

The two TF publishers are part of this target.  wrist_tf needs the vendor
workspace only for ros_robot_controller_msgs; it reads the bus servos and
never commands them.  Check `ros2 run tf2_ros tf2_echo base_link
camera_optical_measured` before trusting wrist observations.
EOF

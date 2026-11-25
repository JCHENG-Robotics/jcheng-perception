#!/usr/bin/env bash
# Record a perception unit failure and emit a journal alert.
#
# Called by jcheng-perception-alert@<label>.service after OnFailure= fired,
# i.e. an ingress, a TF publisher or the fusion exhausted StartLimitBurst and
# systemd stopped restarting it.  Labels: `ingress-<camera>`, `wrist-tf`,
# `ext-tf` or `fusion`.
#
# The robot has no internet access, so this is the alert channel:
#   * journal at user.err (visible in `journalctl -p err`);
#   * /var/lib/jcheng-perception/last-alert.txt and alerts.log (persistent);
#   * a copy under /run/jcheng-perception/ for a running health collector.
# It never restarts the failed unit: after the limit the operator decides.
set -euo pipefail

label="${1:?usage: alert.sh <ingress-<camera>|fusion>}"
case "$label" in
    ingress-*)
        unit="jcheng-perception-ingress@${label#ingress-}.service"
        ;;
    wrist-tf)
        unit="jcheng-perception-wrist-tf.service"
        ;;
    ext-tf)
        unit="jcheng-perception-ext-tf.service"
        ;;
    fusion)
        unit="jcheng-perception-fusion.service"
        ;;
    *)
        unit="$label"
        ;;
esac

state_dir=/var/lib/jcheng-perception
run_dir=/run/jcheng-perception
mkdir -p "$state_dir" "$run_dir"

stamp="$(date -Is)"
nrestarts="$(systemctl show -p NRestarts --value "$unit" 2>/dev/null || echo unknown)"
result="$(systemctl show -p Result --value "$unit" 2>/dev/null || echo unknown)"
action="systemctl reset-failed ${unit} && systemctl start ${unit}"
message="ALERT ${unit}: restart limit hit (NRestarts=${nrestarts} Result=${result}); after fixing the camera/driver run: ${action}"

logger -t jcheng-perception -p user.err -- "$message"

{
    echo "at=${stamp}"
    echo "unit=${unit}"
    echo "label=${label}"
    echo "nrestarts=${nrestarts}"
    echo "result=${result}"
    echo "action=${action}"
} > "${state_dir}/last-alert.txt"
cp "${state_dir}/last-alert.txt" "${run_dir}/last-alert.txt"
echo "${stamp} ${message}" >> "${state_dir}/alerts.log"

# The last 30 lines usually name the failing camera topic / driver in the
# ingress log; keep them next to the alert instead of only in the journal.
systemctl status "$unit" --no-pager --lines=30 > "${state_dir}/last-alert-status.txt" 2>&1 || true
cp "${state_dir}/last-alert-status.txt" "${run_dir}/last-alert-status.txt"

exit 0

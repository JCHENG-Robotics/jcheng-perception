#!/usr/bin/env bash
# Record a GPU unit failure and emit a journal alert.
#
# Called by jcheng-perception-gpu-alert@gpu.service after OnFailure= fired,
# i.e. the GPU unit exhausted StartLimitBurst and systemd stopped restarting
# it.  The laptop has network access, but there is no push channel to the
# robot; the alert lands in the user journal and local state.
set -euo pipefail

label="${1:-gpu}"
unit=jcheng-perception-gpu.service
state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/jcheng-perception"
mkdir -p "$state_dir"

stamp="$(date -Is)"
nrestarts="$(systemctl --user show -p NRestarts --value "$unit" 2>/dev/null || echo unknown)"
result="$(systemctl --user show -p Result --value "$unit" 2>/dev/null || echo unknown)"
action="systemctl --user reset-failed ${unit} && systemctl --user start ${unit}"
message="ALERT ${unit}: restart limit hit (NRestarts=${nrestarts} Result=${result}); after fixing the GPU node run: ${action}"

logger -t jcheng-perception-gpu -p user.err -- "$message"

{
    echo "at=${stamp}"
    echo "unit=${unit}"
    echo "label=${label}"
    echo "nrestarts=${nrestarts}"
    echo "result=${result}"
    echo "action=${action}"
} > "${state_dir}/last-alert.txt"
echo "${stamp} ${message}" >> "${state_dir}/alerts.log"
systemctl --user status "$unit" --no-pager --lines=30 \
    > "${state_dir}/last-alert-status.txt" 2>&1 || true

exit 0

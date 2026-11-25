# 2026-11-25 — split stack under systemd (restart budget + alert)

**Hardware claimed: yes, static only.** Arm stationary. This validates process
supervision, not perception accuracy (see `hardware_20261124_split_edge/` for
the 1.55 mm cross-view result).

## What was verified

The split deployment is no longer started by `split_edge.launch.py`. Five
processes run as individual systemd units in the `ArmPiUltra` container and are
started/stopped through `jcheng-perception.target`:

| unit | process |
|---|---|
| `jcheng-perception-ingress@d435i_ext.service` | external ingress |
| `jcheng-perception-ingress@aurora_wrist.service` | wrist ingress |
| `jcheng-perception-fusion.service` | fusion |
| `jcheng-perception-wrist-tf.service` | bus-servo wrist TF (read-only) |
| `jcheng-perception-ext-tf.service` | external static TF |

Launch `respawn=True` is replaced by `Restart=always` + `RestartSec=2`; the
4 s ingress watchdog still exits 0 on a stalled reader, and systemd creates the
fresh CycloneDDS participant. Each unit has `StartLimitBurst=10` /
`StartLimitIntervalSec=300` and `OnFailure=jcheng-perception-alert@<label>`,
which writes a journal `user.err` entry plus
`/var/lib/jcheng-perception/{last-alert.txt,alerts.log,last-alert-status.txt}`.

## Results

1. **Five units active, target enabled at boot**, `NRestarts=0` after a manual
   start. `systemctl list-units --all 'jcheng-perception*'` shows the target plus
   five running services (`runtime_state.txt`, `installed_units.txt`).
2. **TF frames publish**: wrist `base_link -> camera_optical_measured` at the
   known pose translation `[0.176, 0.009, 0.196]` m (matches the 2026-11-24
   pose A); external composed `base_link -> camera_color_optical_frame` at
   `[0.139, 0.027, 0.037]` m. `wrist_tf` read the bus servos with 0 failures.
3. **Watchdog restart performed by systemd** (`journal.txt`):

   ```text
   09:26:42 [ERROR] d435i_ext: ingress link stalled (frames=75, depth=89); exiting for respawn
   09:26:43 Deactivated successfully
   09:26:45 Scheduled restart job, restart counter is at 1
   09:26:48 edge ready: cameras=d435i_ext target_frame=base_link uplink=True
   ```

   This is the old launch `respawn=True` behaviour, now owned by systemd. The
   wrist ingress also restarted twice after the final start
   (09:28:24 `frames=374 depth=171`, 09:29:06 `frames=317 depth=89`) and
   recovered each time; the known Aurora DDS wedge is still present at roughly
   two restarts per 100 s, i.e. ~6 starts per 300 s, below the 10-start cap.
   If it ever reaches 10/300 s the unit fails and alerts by design, and fusion
   keeps publishing from the external view until an operator resets it.
4. **Stop propagation is clean**: `sudo systemctl stop jcheng-perception.target`
   left 5/5 units `inactive`, `Result=success`, zero markers, zero alerts and
   zero container processes; each wrapper logged
   `stop requested: container exit ... treated as a normal stop`
   (`stop_propagation.txt`, `journal.txt`). Stop settled in 3 s.
5. **Restart budget + alert** on a camera-less instance (does not touch the real
   cameras): `jcheng-perception-ingress@selftest.service` retried a node that
   raises `camera_filter selftest is not in cameras.yaml`; after 10 starts the
   unit entered `failed` at 63 s, and `alert.sh` wrote:

   ```text
   at=2026-11-25T09:27:13+08:00
   unit=jcheng-perception-ingress@selftest.service
   label=ingress-selftest
   nrestarts=10
   result=exit-code
   ```

   plus a `journalctl -p err -t jcheng-perception` entry and the unit's last 20
   status lines (`restart_limit_alert.txt`). `reset-failed` cleared it; the test
   instance and its alert files were then removed. The real ingress/fusion were
   not affected.
6. **Fusion was already confirmed live with TF** earlier in the session:
   `/perception/status` reported `total_frames 121`, `detections_matched 121`,
   `match_success_rate 1.0`, `gpu_alive true`, one confirmed track (09:18:44
   sample). The later sample shows `gpu_alive=false` because the laptop GPU node
   left running from the 2026-11-24 session stopped delivering heartbeats; that
   is the known cross-machine DDS-wedge issue, not this change.

## Problems found and fixed during validation

1. **`set -u` breaks ROS setup**: `source /opt/ros/humble/setup.bash` fails with
   `AMENT_TRACE_SETUP_FILES: unbound variable` under `bash -u`. The container
   preamble uses `set -eo pipefail` only.
2. **A deliberate stop looked like a crash**: SIGTERM makes the rclpy teardown
   exit 1, so systemd marked the unit `failed` and fired `OnFailure`. Marking
   exit 1 as `SuccessExitStatus` fixed the false alert, but a crash would then
   be labelled a success. The first selftest at 80 s had only 8 restarts, so it
   had not yet reached the 10-start limit; a separate transient test
   (`Restart=always` + `/bin/true`, burst 3) hit `Result=start-limit-hit` after
   3 starts in 5 s, proving **clean exits do count** toward `StartLimitBurst`.
   Final design: `ExecStop` touches
   `/tmp/jcheng-perception-<unit>.stopping`, the wrapper translates the node's
   SIGTERM exit to 0 only while that marker exists (or when systemd's own TERM
   is trapped), and `ExecStart` clears stale markers. Crash exit 1 still reaches
   systemd as a failure, preserving `Result`/OnFailure diagnostics; the start
   limit works either way.
3. **Stopping created defunct nodes**: the first ExecStop pattern matched both
   the node and its `ros2 run` parent, orphaning the node into the container's
   `tail` PID 1 (which does not reap). The pattern now matches the node
   executable path only, so the parent reaps it; a stop/start cycle adds zero
   node zombies. About 20 `[perception_*]/[tf] <defunct>` entries left by the
   earlier test iterations remain until the container is restarted; they hold
   no memory and do not match any stop pattern.
4. **Wrapper killed before it could consume the marker**: under a concurrent
   target stop, systemd SIGTERMs the wrapper as soon as the node is gone. The
   fix is the deferred `TERM` trap in every wrapper plus a `$MAINPID` wait in
   `ExecStop`.

## Environment notes

- Pi 5, systemd 252, `ArmPiUltra` container; `armpi-rs-depth` and
  `armpi-vision` were up, so the stack waited for and picked up both cameras.
- Load average during validation was 9–13. The Pi was also running a leftover
  `apt-get -s install -y ros-humble-moveit` (91 % CPU) and
  `realsense2_camera_node` (86 %); the systemd units themselves sat at low CPU.
- The hardware watchdog (`RuntimeWatchdogSec=15`) was already installed.

## Files

- `installed_units.txt` — `systemctl cat` of every installed unit and the
  sha256 of the installed units/scripts/env.
- `deploy_hashes.txt` — sha256 of the repository copies (must match the
  installed ones; `perception.env` matches after the last sync).
- `runtime_state.txt` — final unit counters, in-container processes,
  `/perception/status` sample, TF echoes.
- `stop_propagation.txt` — the final stop/start transcript.
- `restart_limit_alert.txt` — selftest failure output, alert file and status
  dump.
- `journal.txt` — watchdog restart, stop/start journal lines, load/top,
  defunct count.

## How to reproduce

See `commands.md`.

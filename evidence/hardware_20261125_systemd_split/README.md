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

1. **Five units active, target enabled and verified at boot** (2026-11-25
   reboot after install): `jcheng-perception.target` was reached and all five
   services started automatically (`boot_test.txt`). `NRestarts=0` after a
   manual start; the boot counters are in the boot section below.
   `systemctl list-units --all 'jcheng-perception*'` shows the target plus five
   running services (`runtime_state.txt`, `installed_units.txt`).
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
   wrist ingress also restarted repeatedly (5-6 starts in the first 25 min) and
   recovered each time; the known Aurora DDS wedge is still present. On the
   **previous boot the external ingress actually hit the cap**: at 10:08:46 it
   failed with `Result=start-limit-hit` after 10 starts and wrote
   `label=ingress-d435i_ext` (the alert file persisted across the reboot and is
   the best evidence that the budget + alert work on a real ingress, not only
   on the synthetic `@selftest`). On the post-install boot the external ingress
   was at 9 restarts after 25 min, so the current wedge rate can still reach
   10/300 s during a bad phase. Raising the budget is a policy choice; the
   default stays 10/300 s and fusion continues on the wrist view whenever the
   external ingress is down.
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
7. **A camera dead from start is already covered** (`deadcam_test.txt`): a
   temporary config with a `deadcam` external role and nonexistent topics exits
   after ~7 s with `ingress link stalled (frames=0, depth=0); exiting for
   respawn` (rc 0). The state initialises the counters to `-1`, so the first
   watchdog tick reads 0 and marks the link as seen; the 4 s stall deadline then
   applies even with zero real frames. No first-frame-specific code is needed.

## Boot acceptance (2026-11-25, after install)

The host was rebooted after the units were installed (uptime confirmed with
`/proc/uptime`; the Pi RTC makes `who -b`/journal wall times jump, which is why
the journal shows 13:17 for a boot that ended at 14:14 wall time). Results:

- `jcheng-perception.target` was reached at boot; all five units started
  without a manual command.
- `fusion`, `wrist_tf` and both ingresses waited for `armpi-vision` and started
  after the vendor bringup; `NRestarts=0` for fusion/wrist_tf.
- **`ext-tf` had one boot failure**: it started at 13:17:31 with only
  `After=armpi-rs-depth` and was SIGKILLed (status 137) one second later by
  `start_node`'s `~/.stop_ros.sh` (which `kill -9`s every process with "ros" in
  its command line). systemd restarted it at 13:17:34 and it stayed up. The
  unit now also has `After=/Wants=armpi-vision.service` (active only after the
  vendor launch is up, post-kill); a test enforces this for all four
  container-side units. This fix is applied on the running host but has not
  been through a second reboot yet.
- After the boot the two camera drivers were up (`armpi-rs-depth`,
  `armpi-vision`; `aurora930_node` and `realsense2_camera_node` running). The
  laptop GPU node had stopped (SIGINT from the previous session); it was
  restarted manually for this check and then reported `gpu_alive=true`,
  `camera_fps` wrist 2.34, 1 confirmed track, 16 matched detections. It is not
  systemd-managed yet.
- The 28 `<defunct>` entries accumulated before the reboot were cleared (3
  remained after the boot-time/health checks). Load average was 13-14 during
  the check.

## Laptop GPU unit (2026-11-25)

The laptop detector is now a systemd **user** unit
(`~/.config/systemd/user/jcheng-perception-gpu.service`) installed by
`deploy/systemd/laptop/install-laptop-gpu.sh`, with the verified
`/opt/jcheng/models/yolov8s.pt`, a persistent CycloneDDS XML and the
two-camera `gpu_laptop.yaml`. `Restart=always` + `StartLimitBurst=10` /
`StartLimitIntervalSec=300` + `OnFailure` mirror the robot-side policy.

Verified (`gpu_unit.txt`): unit active/enabled, linger on, `detector loaded`
for both cameras; robot `/perception/status` `gpu_alive=true`; restart kept
`NRestarts=0`; stop ended `inactive` with `Result=success` and no alert; start
returned active.

**Reader-wedge watchdog (`gpu_watchdog_test.txt`)**: a second user service
`jcheng-perception-gpu-watchdog` subscribes to `/perception/gpu_status` and
restarts the GPU unit when one camera's counter is frozen for 60 s while
another advances, or the status has been absent for 30 s; both cameras frozen
is treated as an edge-side outage and is not restarted. Budget 5 restarts /
30 min, then `last-watchdog-alert.txt`. Fault injection
(`systemctl --user kill -s STOP jcheng-perception-gpu`) was detected as
`gpu_status stale` after 34 s; the watchdog restarted the unit and
`detector loaded` returned 4 s later, with the robot back at `gpu_alive=true`
and one confirmed track. No alert was written (1/5 restarts).

## Combined status (2026-11-25)

`jcheng-perception-status.timer` runs `jcheng-perception-status.py` every 60 s.
It reads the robot target + five units over SSH, the laptop GPU/watchdog units
and both alert files, writes `~/.local/state/jcheng-perception/status.{json,txt}`
and classifies `overall` as `ok` / `degraded` / `robot_unreachable`.
Verified (`status_aggregator.txt`): the timer refreshed after 54 s; the status
listed 6 robot units + 2 laptop units, all active, reported the 10:08 robot
alert separately and returned `overall=ok`.

## Fixed-point acceptance (2026-11-25, systemd stack)

While the target and all five units were active, a 60 mm AprilTag 36h11 id 0
was measured from both cameras with `scripts/fixed_point_check.py`:

- external: 68 samples / 25.3 s (the tag left the stream for part of the 60 s
  window during the known external ingress wedge); `tf=exact` for all samples.
- wrist: 158 samples / 59.6 s; `tf=exact` for all; within-view depth sd
  1.08 mm, PnP sd 1.11 mm.
- raw cross-view: depth 13.33 mm / PnP 10.44 mm — **FAIL** (chain offset).
- with the deployed `config/view_bias.yaml` external offset
  (+9.9, -1.8, +9.4) mm: depth **3.32 mm PASS**, PnP **5.50 mm FAIL**
  (0.5 mm over the 5 mm gate).

The production depth path (what the edge back-projects and fusion consumes)
passes. The PnP residual is the known wrist depth-vs-PnP bias (~7.2 mm in z)
plus the curled tag sheet; the 2026-11-24 run quoted 3.67/3.07 mm with a
flatter tag. Flatten/reposition the tag and repeat before quoting a <5 mm
absolute-accuracy claim. This is the defined-point camera/TF gate, not a
tag-fusion recording: the systemd GPU unit runs the COCO detector, so the
tag-fusion reference remains split run 4 (cross-view 1.55 mm, fusion max
2.14 mm). Files: `fixed_point_20261125.txt`, the two CSVs and the raw/bias
report JSONs.

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
   node zombies. The entries left by the earlier test iterations were cleared
   by the 2026-11-25 reboot (3 remained from the boot-time health checks).
4. **Wrapper killed before it could consume the marker**: under a concurrent
   target stop, systemd SIGTERMs the wrapper as soon as the node is gone. The
   fix is the deferred `TERM` trap in every wrapper plus a `$MAINPID` wait in
   `ExecStop`.
5. **`ext-tf` started inside the vendor kill window** (boot test): with only
   `After=armpi-rs-depth` it started in the same second as `start_node`, whose
   `~/.stop_ros.sh` `kill -9`s every process with "ros" in its cmdline; the unit
   died with status 137. It now also has `After=/Wants=armpi-vision.service`
   and `tests/test_systemd_deploy.py::test_container_units_wait_for_the_vendor_bringup`
   enforces that dependency for all container-side units. Applied on the host;
   a second reboot has not been done.

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
- `boot_test.txt` — post-install reboot: target/unit starts, the `ext-tf`
  status-137 boot failure, the real `start-limit-hit` alert from the previous
  boot, counters, GPU status after the manual laptop restart, TF and defunct
  count.
- `deadcam_test.txt` — fake-camera test proving the existing 4 s watchdog
  exits an ingress whose camera never produced a first frame.
- `gpu_unit.txt` — laptop GPU user unit: state/enable/linger, detector load,
  restart/stop/start lifecycle and the robot-side `gpu_alive` check.
- `gpu_watchdog_test.txt` — SIGSTOP fault injection: status stale detected,
  watchdog restart, detector reload and robot recovery.
- `status_aggregator.txt` — timer state, combined `status.txt`/`status.json`,
  and the 54 s automatic refresh.
- `fixed_point_20261125.txt`, `fixed_point_{ext,wrist}_20261125.csv`,
  `fixed_point_report_{raw,bias}_20261125.json` — systemd-stack fixed-point
  run and the 5 mm gate (depth 3.32 PASS / PnP 5.50 marginal FAIL).

## How to reproduce

See `commands.md`.

# 2026-11-24 — fixed-point (<5 mm) validation: protocol ready, blocked by a silent bus-servo bus

Robot: ArmPi Ultra, Pi 5 (ROS 2 Humble, container `ArmPiUltra`), laptop RTX 3060
(ROS 2 Jazzy). **No motion commands were sent.** The only actuator test was one
buzzer pattern; it is recorded below.

## Goal and protocol

Validate that one physically defined point on the table is measured in
`base_link` to better than 5 mm:

1. **Wrist multi-pose** — the same AprilTag seen from 3+ arm poses must give the
   same `base_link` point (this constrains the full hand-eye translation, which
   the earlier table-plane check did not).
2. **Cross-view** — the external D435i measures the same tag; wrist vs external
   agreement checks both extrinsic chains.

Measurement per sample (`scripts/fixed_point_check.py`):

- `depth` — production path: AprilTag centre pixel → median depth patch → RGB
  `CameraInfo` → `base_link` TF at the depth stamp (same steps as
  `jcheng_perception_core.backproject`).
- `pnp` — RGB-only `solvePnP(IPPE_SQUARE)` marker centre, independent of depth.
- Target: existing printed AprilTag 36h11, ID 0, 60 mm
  (`jcheng-manipulation/armpi_ultra_ws_clean/docs/calibration/armpi_apriltag_36h11_id0_60mm.pdf`),
  the same sticker the manipulation project validated on 2026-11-16/18.
- Pose changes are detected from the camera origin (>10 mm) and tagged with
  `pose_seq`; the CSV keeps every sample so travel is dropped in analysis.

Self-test on the robot's OpenCV 4.10.0-dev (synthetic marker at known poses):

| distance | rotation | range error | reprojection |
|---|---|---|---|
| 0.20 m | 0° | **0.54 mm** | 0.11 px |
| 0.35 m | +20° | 2.05 mm | 0.03 px |
| 0.50 m | −35° | 2.11 mm | 0.05 px |

The 2 mm figures are pixel quantisation of a 71/50 px synthetic marker, not
detector bias (reprojection is <0.05 px). The self-test tolerance is 5 mm, which
still catches corner-order/size mistakes (tens of mm).

## Verified before the blocker

- Both cameras stream: external `640x480` colour + aligned depth (same stamp,
  `camera_color_optical_frame`); wrist `640x400` (`rgb_camera_link` /
  `depth_camera_link`).
- External chain: `ext_tf` + driver TF resolves
  `base_link → camera_color_optical_frame`; matrix matches `frames.yaml`
  (translation 0.139, 0.027, 0.037 m).
- Scene: an orange can sits directly in front of the external D435i (its depth
  is largely closer than the 0.15 m minimum; 30.6 % valid pixels), so the tag
  must go on the table to the left of the can for cross-view.
- `config/wrist_tf.yaml`, `frames.yaml`, `ext_tf.yaml`, `cameras.yaml` and the
  script hashes: see `hashes.txt`.

## Blocker: every bus-servo read times out

`wrist_tf` needs `base_link → camera_optical_measured` from bus-servo positions.
The vendor `bus_servo/get_state` service returned nothing, so `wrist_tf` never
published and no wrist measurement is possible.

Timeline (Pi-clock times; the Pi clock jumped ~19 min during the session, so
only ordering is reliable):

| time | event |
|---|---|
| 14:12:39–40 | last successful `bus_servo_read_position` lines in the vendor journal (burst of ~4 servos at ~10 ms, matching the 8766 motion gateway's post-command verification) |
| 14:23+ | first single-servo read hangs; `wrist_tf` starts, parses the URDF chain, then never publishes |
| 14:29, 14:32 | `start_node` restarted twice (clears the in-node read lock); fresh reads still hang |
| 14:35 | full robot power cycle (Pi reboots, servo LEDs go dark) — reads still hang |
| later | 8766 gateway stopped, controller restarted, unicast id 6 read — still hangs |
| later | buzzer `{freq: 2000, on_time: 1.0, off_time: 0.4, repeat: 3}` — **audible, operator confirmed 3 long beeps** |

Discriminators:

- Battery telemetry proves only MCU→host: `Board.get_battery()` drains an
  unsolicited `sys_queue`, it is not a request/response.
- The buzzer proves **host→MCU commands work**.
- Servo LEDs are lit with the robot on and go dark when the main switch is off,
  so the MCU and servo rail were both reset by the power cycle.
- Unicast (id 6) and broadcast (254) reads both hang; the broadcast test is weak
  (simultaneous replies collide) but the unicast one is not.
- Only `ros_robot_controller` owns `/dev/ttyUSB0`; no competing serial reader.
- The SDK read is `bus_servo_queue.get(block=True)` **with no timeout**, and the
  service callback holds the single-threaded executor, so one failed read also
  starves every other callback (LED/buzzer/telemetry) until the node restarts.

Conclusion: the fault is on the **MCU→servo-bus side** — the 3-pin bus cable /
connector, or the MCU's servo UART (USART2/`BUS_EN` PA1 direction control). It is
not a ROS, TF or bridge issue, and it did not clear with a full power cycle.

## Next actions (physical, then resume)

1. Power off. Reseat the 3-pin bus cable at the expansion board
   (`总线舵机`/BUS SERVO port) and at the base servo; inspect the signal wire.
2. If a spare bus cable exists, swap it; if the arm has a separate controller,
   test the arm on it to separate cable from HAT.
3. Power on and tell the assistant; the first check is one unicast read
   (`id 6, get_position`) before anything else.
4. Only then: start `wrist_tf` + `ext_tf`, place the tag flat on the table left
   of the can, and run `fixed_point_check.py --camera wrist` for 3 poses plus
   `--camera ext` for the static cross-view.

## State left on the robot

- Robot is powered **off** at the end of the session (laptop's USB-Ethernet link
  is down, Pi 10.42.0.218 not reachable).
- `armpi-gateway-pose` (8766) was stopped for the diagnosis; it is a manual
  service, start it only if the Studio pose/motion tool is needed.
- After the vendor restarts, `armpi-vision`'s Aurora node is dead while the unit
  still shows `active (exited)`; restart it once per the documented procedure
  before any wrist-camera run.
- `ext_tf` / `wrist_tf` are not running. `/tmp/fixed_point_check.py` exists in
  the container; the source of truth is `scripts/fixed_point_check.py`.

## Files

- `snap_ext_rgb.jpg`, `snap_ext_depth_preview.png` — external view (can blocks
  the centre).
- `snap_wrist_rgb.jpg`, `snap_wrist_depth_preview.png` — wrist view at ~0.35 m.
- `hashes.txt` — config and script hashes.
- Measurement/report scripts (repo): `scripts/fixed_point_check.py`,
  `scripts/fixed_point_report.py` (segments by pose, applies the 5 mm gate and
  writes the JSON summary).

## Run 2 (resumed after the cable was reseated) — external half only

`fixed_point_ext_20261124_run2.csv`, `summary_ext_run2.json`,
`hashes_run2.txt`. No motion command was sent; two bus-servo **write** probes
were (id 4, 784 → 814 → 784, 2 s each; then id 6 base yaw 500 → 560 → 500,
3 s each). The operator confirmed the base yaw swung ~14° and returned, so
**MCU → servo transmission works**; the id 4 7° nudge was not clearly visible.

**Correction: the bus was never silent — the client was on the wrong RMW.**
Every read this session that "timed out" was issued from a `bash -c` shell that
sourced only `/opt/ros/humble/setup.bash` + the vendor workspace, i.e.
`rmw_fastrtps_cpp`. The vendor stack and the operator's scripts run under
`~/.zshrc`, i.e. `rmw_cyclonedds_cpp` + `CYCLONEDDS_URI`. **Topic publish does
cross the two RMWs** (the base-yaw probe really moved the arm), **ROS 2 services
do not** — so the request never reached `ros_robot_controller` and the client
waited until its own timeout. Same node, same minute: the `bash` client times
out, the operator's `zsh` snippet returns `success=True, positions=[[501]]` in
under 10 ms.

The operator's `strace` of the node shows the full path working: query
`AA 55 05 02 05 06 EC` → reply `AA 55 05 05 06 05 00 F6 01 52` (id 6, cmd 5,
position 502) in 7.65 ms, and a five-servo call returned 502/596/830/113/501,
matching the ROS service. Earlier conclusions in this bundle that the
`servo → MCU` receive side was dead, and that the node was wedged, are
**withdrawn** — `/ros_robot_controller/describe_parameters` "hung" only because
it was called from the same wrong-RMW shell. A `ros2` daemon started under one
RMW also keeps reporting the graph, so `ros2 service list` can show a service
that cross-RMW calls still cannot reach.

Two real traps remain. (1) The SDK read is `bus_servo_queue.get(block=True)`
with no timeout and no check that the returned id/command match the request — a
genuinely lost reply would block the service callback and the single-threaded
executor forever. It was not triggered in this session, but a timeout should be
treated as "preserve the scene and strace the node", not "retry". (2)
`/controller_manager/servo_states` republishes the last commanded pulse at
~10 Hz with `voltage: 0`, so the ArmPi Studio position readout is that cache,
not bus feedback.

**Script bug found and fixed.** `fixed_point_check.py` recorded `detections=0`
on a tag the detector finds in a saved frame. One `spin_once` per sample period
left the callback queue behind the streams, so the colour frame paired with the
depth ring aged past `--depth-tolerance-ms` (250 ms) and every sample was
dropped *before* detection. The loop now spins continuously (50 ms) and samples
only when the period is due: 0/24 samples before the fix, 19/19 after, same
scene. The script had never run on hardware before (the first session was
blocked before it started).

**External static measurement** (60 s, 3 Hz, tag flat on the table 0.217 m from
the D435i, `tf=exact` for all 175 samples):

| estimate | point in `base_link` (mm) | 3D sd |
|---|---|---|
| depth (production path) | (+314.83, +52.36, −92.49) | 0.48 mm |
| PnP (RGB only) | (+318.87, +52.28, −92.98) | 0.03 mm |
| depth − PnP | (−4.04, +0.07, +0.49) | 4.07 mm norm |

Marker reprojection 0.85 px median. The 4.07 mm is a depth-vs-PnP bias of this
one camera (3.7 mm of it along camera z, depth closer), not a cross-view error;
it does consume most of the 5 mm budget if both estimators are compared across
views. **The cross-view half is not evaluated**: it needs
`base_link → camera_optical_measured`, which needs bus-servo reads. So this run
proves within-run repeatability and extrinsic-chain composition, not absolute
accuracy.

Next: run the wrist 3-pose half with `wrist_tf` and the check script started
from a `~/.zshrc` (CycloneDDS) shell, then apply the 5 mm gate with
`fixed_point_report.py`. Every client on the robot must be on
`rmw_cyclonedds_cpp`; a hanging service call is a suspected RMW mismatch first,
a hardware fault second.

## Run 3 — the gate ran, and the cross-view half fails

`fixed_point_wrist_20261124_run3.csv`, `fixed_point_ext_20261124_run3.csv`,
`report_run3.json`, `summary_run3.json`, `hashes_run3.txt`. With the bus reads
working (`wrist_tf` read at 10 Hz, 0 failures over ~3200 reads), the protocol
finally ran end to end. The tag was physically moved between run 2 and run 3,
so the external point was re-measured at its current position before the wrist
run.

**Wrist multi-pose — PASS.** Four settled poses, `joint1` (id 6) yaw only:
501 → 536 → 466 → 501 ticks (±8.4°), which swept the camera along y from +24.2
to −4.2 mm with x/z constant. 187/187 samples, `tf=exact`, 0 TF failures.
Pairwise agreement depth 0.76–4.03 mm, PnP 0.28–3.77 mm — all under 5 mm.
Limitation: every pose differs only in joint1, so this constrains the base yaw
and the hand-eye rotation, not the hand-eye translation.

**Cross-view — FAIL.**

| estimate | wrist − external (mm), per pose | norm |
|---|---|---|
| depth | (+9.9…+10.1, −1.8…+2.0, +9.4…+10.6) | 13.9–14.3 mm |
| PnP | (+9.0, −1.7…+2.1, +5.1…+5.2) | 10.4–10.6 mm |

The offset is **constant across the 28 mm pose baseline** (x varies <1 mm), so
it is an extrinsic-level translation, not hand-eye noise. Prime suspect is the
external chain: it also places the tag ~7 mm below the table plane the wrist
measures at −85.9…−86.8 mm, while the wrist places the tag ~3 mm above it
(paper thickness plus its own 5–6 mm depth-vs-PnP bias). Not excluded: the
wrist hand-eye translation (joint1-only baseline), and a wrong external
`CameraInfo` K, which would shift the external depth *and* PnP together and is
untested.

Also fixed: `fixed_point_report.py` raised `NameError: limit` on the cross-view
path — that branch had never been executed on data. Use `args.limit`.

Next: extend the wrist poses to joint2/joint4 moves that keep the tag in frame
(verify with a snapshot per pose) so the hand-eye translation is constrained;
re-check the external extrinsic (2026-11-20 chessboard) and the mount; compare
the external `CameraInfo` K with the driver's advertised calibration. Do not
tune a single number to make the gate pass.

## Offline constraint (2026-11-24, no motion) — what the old data already says

`offline_constraint_20261124.json`. No actuator command was sent; the robot
stayed at the run-3 measurement pose (pulses 496/592/830/108/501, 7.50 V).

**The hand-eye translation is already constrained to a few mm, and the
rotation is the weaker parameter.** Two independent multi-pose chessboard
solves agree in translation but not in rotation:

| method | Δtranslation servo − table_down | Δrotation |
|---|---|---|
| TSAI | (+2.91, +0.98, +0.60) mm, 3.13 mm | **2.49 deg** |
| PARK | (+1.31, +2.01, −1.44) mm, 2.97 mm | 0.69 deg |

At the tag (0.30 m) one degree of hand-eye rotation is 5.2 mm, so the TSAI
difference alone is ~13 mm — the size of the cross-view failure. Run 3 moved
only joint1, which makes the 9-parameter (point + 6-DoF hand-eye) model rank
7; it could not test this. The planned joint2/joint4 set below is rank 6/6
with 1σ = 2.3–3.3 mm translation and 0.6–0.9 deg rotation at 1 mm per-pose
noise, so it can decide between the two fits.

**Two concrete chain inconsistencies, both smaller than but comparable to the
observed offset:**

1. `config/frames.yaml` deploys the **provisional** 2026-11-20 external
   extrinsic (139.30, 27.04, 36.75 mm, pitch 32.62 deg) while labelling the
   refined evidence file as its source; the adopted refined result is
   (136.49, 27.13, 38.41 mm, 31.56 deg). At the tag this is a 4.5 mm shift.
2. The external extrinsic was solved with the **table-down** hand-eye; the
   deployed wrist chain (`wrist_tf.yaml`) uses the **servo** hand-eye. Feeding
   the run-3 tag vector through both predicts wrist − external offsets of
   15.9 mm (table-down TSAI), 7.5 mm (table-down PARK) — the observed PnP
   offset is 10.4–10.6 mm, depth 13.9–14.3 mm.

So a physical mount shift is **not required** to explain the cross-view
failure; the two chains were never built on the same calibration. It is not
excluded either — the multi-DoF wrist run is still the clean test, followed by
re-deriving the external extrinsic from the *deployed* hand-eye and the
adopted (not provisional) numbers.

**Intrinsics are not the problem.** The live wrist `CameraInfo` matches the
documented K/D exactly. The live external stream is 640x480 RealSense colour
with the factory K (fx 606.50, fy 606.60, cx 325.03, cy 249.19, D = 0); the
2026-11-20 UVC calibration is 1280x720 (fx 892/903), a different colour mode.
They must not be mixed, but the extrinsic is resolution-independent.

**Ready to run (not executed):** `scripts/fixed_point_check.py` now writes the
full base←camera rotation (`rot_r00..rot_r22`, self-test unchanged), and the
new `scripts/handeye_translation_check.py` fits the 6-DoF correction from the
CSV (synthetic test recovers an injected 2.5 deg / (2, −1, 3) mm exactly).

| pose | joint1..4 pulses | tag px | margin | rotation from live |
|---|---|---:|---:|---:|
| P0 live | 496, 592, 830, 108 | — | — | 0.0 deg |
| P1 | 600, 548, 875, 120 | (481, 108) | 41 px | 30.9 deg |
| P2 | 586, 643, 851, 120 | (429, 295) | 44 px | 23.9 deg |
| P3 | 471, 548, 875, 129 | (270, 105) | 40 px | 17.3 deg |
| P4 | 471, 643, 863, 139 | (277, 298) | 42 px | 13.2 deg |

Every target is inside the adopted pulse bounds, the end effector stays
≥152 mm above the table, and the whole run (moves + 25 s holds) is ~3.5 min.
Do not adopt a corrected hand-eye from this until the run is repeated and the
correction is consistent across both the depth and PnP estimators.

## Run 4 — multi-DoF wrist poses: the drift is not a hand-eye error

`fixed_point_wrist_multidof_p0..p4.csv` (one CSV per settled pose),
`fixed_point_wrist_multidof_merged.csv`, `handeye_multidof_20261124.json`,
`handeye_candidates_20261124.json`, `hashes_multidof_20261124.txt`. The only
motions were the supervised `move_to_pose.py` targets below; the arm started
and ended at the run-3 measurement pose.

| pose | joint1..4 pulses (command) | tag px | samples | PnP point in base (mm) |
|---|---|---:|---:|---|
| P0 home | 496, 592, 830, 120 | (313, 236) | 21 | (301.87, 6.25, −86.76) |
| P1 | 600, 548, 887, 149 | (469, 110) | 36 | (305.73, 12.38, −82.40) |
| P2 | 586, 643, 875, 129 | (427, 245) | 39 | (305.93, 14.02, −82.20) |
| P3 | 500, 548, 851, 120 | (315, 120) | 33 | (301.81, 5.45, −85.69) |
| P4 | 529, 643, 887, 149 | (343, 237) | 28 | (307.14, 9.36, −81.46) |

Rotation spread from P0: **29.5, 22.1, 15.9, 7.4 deg**; every pose detected the
tag in 100 % of samples and `tf=exact` throughout.

**Result: the same physical tag moves by 3D RMS 4.6 mm (PnP) / 4.5 mm (depth)
across the five poses, ~9 mm peak to peak in y.** Within-pose spread is at the
CSV's 1 µm quantisation, so this is systematic, not noise.

The ensemble of chessboard hand-eye fits does **not** explain it:

| hand-eye used | depth RMS | PnP RMS |
|---|---:|---:|
| deployed servo TSAI | 4.54 mm | 4.61 mm |
| servo PARK | 4.26 mm | 4.33 mm |
| table_down TSAI | 3.93 mm | 4.04 mm |
| table_down PARK | 3.98 mm | 4.04 mm |

The best candidate (table_down) removes only ~0.6 mm. A free 6-DoF correction
fits the five points better (residual 2.6 → 1.1 mm) but is ill-conditioned
(1σ = 1 deg / 3.5 mm) and extrapolates badly: applied to the run-3 pose
(j3/j4 just outside the fitted range) it moves the wrist point by 20–30 mm.
**Do not adopt it.**

Interpretation: the 29 deg pose set now constrains the wrist chain, and the
residual is a joint-level / target effect (backlash or compliance in the
servo-to-link path, or the printed tag not being a perfect plane), not the
hand-eye translation. Run 3's 10–14 mm cross-view offset therefore cannot be
attributed to the hand-eye alone. The decisive tag-independent test is the
table-plane check at these same poses: a joint-level error tilts/shifts the
table plane the same way, a hand-eye error would have shown up in the
candidate table above.

Operational notes for the next run:

- Do not run the capture concurrently with a move: the capture process
  wedged after the arm moved (state `futex_wait`, no rows, `tf_failures=0`).
  Move first, wait ~4 s for the colour/depth stream to recover (one 22 s run
  right after a move produced 1/33 samples), then capture.
- `move_to_pose.py --pose -0.016,...` fails silently (argparse eats the
  leading minus). Use `--pose=-0.016,...`.
- The servos settle 2–8 pulses off target; the framing check must allow that
  (the design used ±8 ticks and kept ≥22 px margin).

# 2026-11-24 — defined reference point (AprilTag) validation

**Hardware claimed: yes, static only.** Arm not commanded; the 60 mm AprilTag
36h11 id 0 on the table was used as the common physical point. Goal: replace
the object-dependent bbox centre with a point that both views measure
identically, so the cross-view residual and the first-pair reference step are
millimetres instead of the cup's 57-67 mm.

Deployed edge: `eed1ca9` (no edge code changes for this mode). GPU: new `aruco`
backend, `config/gpu_tag.yaml`, labels `reference_tag_<id>`, centre keypoint
first, four corners at lower confidence. The existing edge keypoint depth path
back-projects the centre; the bbox is only a fallback.

## Runs (three attempts, 2026-11-24)

| run | result | reason |
|---|---|---|
| 1 | **0 observations** | edge did not reach `edge ready` inside the 60 s `timeout` (Pi load ~9.6 during an SD-I/O burst); the SIGINT landed in the Python import |
| 2 | **wrist only** (30 tag observations) | laptop GPU external uplink reader wedged: `frames_received` 2 external vs 246 wrist |
| 3 | **both views** (311 observations: 120 external / 191 wrist, 40 s) | edge started ready-gated, then the GPU node was restarted while the edge was publishing; external reader recovered (21/41 frames at the check) |

Run 3 is the validation data. The successful recipe: start the edge, wait for
`edge ready`, restart the GPU node, then record. The plain "GPU first, then
edge" order reproduced the external-reader wedge.

## Defined-point result

Per-view tag centre means over run 3 (already chain-corrected by the edge):

| view | mean (mm) | σ (mm) | observations |
|---|---|---:|---:|
| external | (330.5, −14.3, −82.8) | 0.1 / 0.0 / 0.1 | 120 |
| wrist | (330.3, −12.3, −79.6) | 0.4 / 0.9 / 0.6 | 191 |
| **external − wrist** | **(+0.2, −2.0, −3.2)** | | **3.77 mm** |

The replay learns `Track.view_offset = (+0.03, −1.93, −3.31) mm`, matching the
measured difference. That is the defined-point residual after the chain
correction, and it is what replaces the cup's 57-67 mm.

Independent fixed-point check (`fixed_point_check.py`, 20 s per view, 36/30
samples, `tf=exact`, reprojection 0.21 / 0.75 px), 5 mm gate:

| comparison | raw | after `config/view_bias.yaml` (+9.9, −1.8, +9.4 mm on external) |
|---|---:|---:|
| depth (wrist − external) | 15.92 mm FAIL | **3.67 mm PASS** |
| RGB PnP (wrist − external) | 12.79 mm FAIL | **3.07 mm PASS** |

The raw value is the chain bias; the corrected value is the defined-point
agreement. Per-view depth-vs-PnP bias remains: external 4.2 mm, wrist 7.4 mm
(the tag sheet is curled and the wrist sees it more obliquely). The absolute
tag centre in `base_link` is wrist PnP (335.4, −12.7, −85.7) mm; corrected
external PnP is (334.7, −14.7, −83.5) mm. The table reference is −82.5 mm, so
the tag centre sits ~3 mm above it; the printed sheet is not flat.

## Fused-track behaviour

- No 67 mm first-pair step: the replay track starts at t≈0.9 s on the external
  point (330.6, −14.3, −82.9) mm and moves to the wrist reference by only the
  3.8 mm residual; the live recorded track starts at (329.7, −13.1, −78.2) mm,
  within 3.8 mm of both view means.
- Live recorded fused track (edge `eed1ca9`, before the min-dt fix below):
  max 11.4 mm, p95 6.2 mm, final 3.7 mm.
- Offline replay (`--source-bias none`, which is required because the edge
  already applied `view_bias.yaml`) with `velocity_min_dt_s = 0.10`:
  max **8.3 mm**, p95 5.4 mm, final 3.6 mm.
- The remaining few mm are tracker transients, not the reference switch.
  The tag run exposed near-duplicate capture stamps (47/191 wrist observations
  had dt ≤ 0); a millimetre difference over 0.04 s becomes ~25 mm/s and a
  10 mm extrapolation. `velocity_min_dt_s = 0.10` rejects such baselines; it
  was added **after** the live runs and is covered by tests/replay, not yet by
  a new hardware run.

## Run 4 — min-dt rerun (partial: external only)

After deploying `f90b5d5` (installed `velocity_min_dt_s = 0.1`), a 60 s bounded
rerun was attempted. It did **not** reproduce the dual-view window:

- 45 external tag observations only (t = 0.3–14.3 s), **0 wrist**; the edge
  reported `aurora_wrist:depth_failures=365`. The GPU received 117 external /
  615 wrist frames, but the external tag stream stopped after ~14 s and no
  wrist observation survived depth matching.
- External tag mean (330.5, −14.2, −82.9) mm, σ 0.2/0.1/0.1 mm — the same tag
  point as run 3.
- Fused external-only track: max **2.42 mm** over the whole span, **0.38 mm**
  after a 6 s settle (report 0.4 mm), final 0.08 mm; biggest single step
  2.6 mm at t = 4.4 s. No velocity transients.
- Ops: the edge external reader had to be recovered by restarting
  `armpi-rs-depth`; the external fps went 0 → 0.64 → 3.7. The GPU external
  uplink reader then stopped after ~14 s again. The dual-view rerun is blocked
  by these two reader/depth-matching issues, not by the min-dt change.

Interpretation: the deployed min-dt code is stable on the external tag track
(0.4 mm), but this is **not** a dual-view repeat of the 11.4 mm → ~8 mm
comparison. Run 3 remains the dual-view defined-point result (3.77 mm,
5 mm gate 3.67/3.07 mm PASS); a wrist-depth/reader fix is needed before the
dual-view min-dt confirmation can be repeated.

## Run 5 — GPU restart harness + Aurora restart (blocked by edge intake)

To attack both reader issues, the Aurora driver was restarted (stopped the
service, killed the orphan `aurora930_node` 5529 holding the USB device, started
the service once) and the GPU tag node was run in a **10 s restart loop** (7
windows) so its external uplink reader never stayed wedged. A fresh edge ran
with `f90b5d5`.

- **Wrist depth matching improved:** run 5 reported `aurora_wrist:depth_failures=1`
  (run 4: 365), so the Aurora restart fixed the depth side.
- **But the edge intake stalled after ~6 s:** 18 external tag observations
  (t = 0.2–5.6 s), 0 wrist; edge status at t = 57 s: 88 frames, 27 detections,
  25 matched, `gpu_alive=false`. All 7 GPU tag windows loaded, so the GPU was
  not the blocker; the edge stopped receiving/processing frames.
- External-only fused track: max 1.79 mm, final 1.31 mm (report 1.8 mm).
- Pi load after the driver restarts rose to 17–23 (D435i node ~87 %, gateway
  ~23 %, Aurora ~24 %, plus zombie entries from the container's non-reaping
  PID 1). No further hardware retries were made.

Conclusion: the wrist depth matching problem is fixed; the remaining blocker is
the edge's camera/detection intake under Pi load / the recurring reader stall.
The dual-view min-dt confirmation is still not obtained; run 3 remains the
dual-view defined-point result.

## Run 6 — same-distro Humble GPU (best result so far)

The user pointed out the laptop's Humble Docker image. A dedicated container
(`jcheng-humble-tag`, image `armpi-humble-moveit:latest`, host network, wired
CycloneDDS config) was created, `python3-opencv` 4.5.4 (with ArUco) installed,
and `jcheng_perception_msgs/core/gpu` built in Humble (`/workspace/tag_ws`).
Both the laptop GPU node and the Pi edge are now **ROS 2 Humble**, removing the
cross-distro DDS path.

Two edge fixes were deployed first (working tree, not committed):

- `uplink_allowed()` resets the per-camera uplink baseline after a backwards
  clock jump (the Pi clock can jump after NTP; a future baseline silently
  skipped that camera's uplinks forever). Tests: `tests/test_uplink_rate.py`.
- `UplinkWorker` now catches any encode/publish exception, counts it and keeps
  the thread alive instead of dying silently; the status notes now expose
  `encode_failures` and `rate_limited`.

Run 6 (60 s, Humble GPU tag node, recorder also in Humble):

- 141 wrist tag observations, continuous t = 0.5–59.8 s; **0 external**.
- Fused **wrist-only** track: max **7.30 mm**, p95 4.66 mm, final 3.93 mm
  (another early track 10.03 mm). This meets the ~8 mm target numerically, but
  it is **not a dual-view** confirmation.
- Edge status: 468 frames, 410 detections, 234 matched, `gpu_alive=true`;
  notes `d435i_ext:depth_failures=13, rate_limited=11`,
  `aurora_wrist:depth_failures=161, rate_limited=27`.
- External path: GPU received 42 external frames (wrist 376), but no external
  tag observations. A current external frame still contains the tag
  (419.5, 273.2) and aligned depth at the centre is 0.225 m; a 12 s check saw
  no external uplink messages, so the external uplink/reader still stops after
  the first window. This is not the model or the Humble DDS distro mismatch.

**Humble container recipe (for future runs):**

```bash
docker run -d --name jcheng-humble-tag --network host \
  -v /home/jerichocheng/jcheng/humble_ws:/workspace \
  -v /home/jerichocheng/jcheng/jcheng-perception:/repo:ro \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///workspace/cyclonedds-wired.xml \
  -e ROS2CLI_NO_DAEMON=1 -e HOME=/tmp \
  armpi-humble-moveit:latest sleep infinity
docker exec jcheng-humble-tag bash -lc 'apt-get update -qq && apt-get install -y -qq python3-opencv'
# ln -s /repo/ros2_ws/src/jcheng_perception_{msgs,core,gpu} /workspace/tag_ws/src/
docker exec jcheng-humble-tag bash -lc 'source /opt/ros/humble/setup.bash; cd /workspace/tag_ws && colcon build --symlink-install --packages-select jcheng_perception_msgs jcheng_perception_core jcheng_perception_gpu'
```

## Watchdog diagnostic (2026-11-24, reverted)

A subscription watchdog was added after run 6: if a camera's frame or depth
counter did not advance for 4 s, the edge destroyed and recreated that
camera's image/depth/CameraInfo subscriptions (10 s cooldown). It was tested
with the Humble tag GPU and then reverted.

```text
d435i_ext: frames=5 stuck, depth 31 -> 75 -> 117 (advancing); recoveries=3
aurora_wrist: frames 40 -> 80 -> 123 (advancing), depth=7 stuck; recoveries=3
```

Recreating the links did **not** recover either stream. The two stuck streams
are exactly the ones needed for tag observations (external colour and wrist
depth). Note: the host Jazzy YOLO node and the Humble tag node were both
publishing during this diagnostic, so its detection counters (5 matched / 134
dropped) are not clean; the per-camera frame/depth counters are.

Conclusion: the wedge is at the **DDS participant/reader** level, not the
subscription object; recreating subscriptions in the same process cannot
recover it. The watchdog was reverted (the Pi runs the committed `2a33cbb`
uplink-clock-jump/worker fix only). The next fix should isolate the camera
readers (separate participants/relay per camera) or address the CycloneDDS
version/configuration.

## Runs 7-9 — root cause: two gateway consumers, not the perception code

After the Humble and uplink fixes the external reader still stopped while the
wrist continued. Two independent gateway-side consumers were responsible:

1. **Aux gateway (8765) used Fast DDS.** `armpi-gateway.service` did not pass
   `RMW_IMPLEMENTATION`/`CYCLONEDDS_URI`, so this gateway subscribed to
   `/camera/color/image_raw` with Fast DDS while the camera driver and edge use
   CycloneDDS. Large colour samples across RMWs wedged the external reader.
   Fixed by patching `/usr/local/lib/armpi-gateway/armpi-gateway-run.sh` to
   pass `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
   `CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml` and `ROS_DOMAIN_ID=0`
   via `docker exec -e`, and adding the same lines to
   `/etc/armpi-gateway/gateway.env`. Backups:
   `/home/pi/armpi-gateway-rwm-20261124/`.
2. **Motion gateway (8766) was an orphan process** (PPID 0, service inactive)
   subscribed to `/depth_cam/depth_registered/points`. It kept blocking the
   Aurora node's depth publication; the wrist depth reader stayed wedged even
   after a fresh edge. Killing PID 427260 recovered wrist depth.

| run | change | observations | cross-view | fused track |
|---|---|---|---|---|
| 7 | aux gateway Cyclone | 225 external, 0 wrist | n/a | external-only max 4.82, p95 2.53, final 0.08 mm |
| 8 | motion service "stopped" but orphan still running | 198 external, 0 wrist | n/a | external-only max 3.58, p95 2.21, final 0.24 mm |
| 9 | orphan motion gateway killed + fresh edge | **240 external + 195 wrist** | **3.37 mm** | **max 10.03, p95 3.78, final 1.19 mm** |

Run 9 is the **dual-view confirmation**: both tag streams were continuous for
60 s (0.1-59.9 s / 0.5-60.1 s), edge 1220 frames / 580 matched,
`gpu_alive=true`. Offline replay (`--source-bias none`) gives max 9.9 mm, p95
3.6 mm, final 1.4 mm and a learned `view_offset` of (+0.48, -1.21, -2.98) mm,
consistent with the measured cross-view (-0.04, -1.73, -2.90) mm. The worst
sample is a 12.4 mm **view-switch transient** at t = 26.2 s
(external -> wrist); p95 and final are well inside the ~8 mm target, the max
is slightly above it. No 67 mm reference step.

Operational state after the test: the aux gateway runs CycloneDDS (patched).
The 8766 motion gateway was stopped during run 9, but at the final check
`armpi-gateway-pose.service` was **active again** (systemd/ops restarted it).
Its pointcloud subscription is the wrist-depth blocker, so a future perception
run can hit the wedge again until that subscription/QoS is changed or the
gateway is stopped for the run.

## Ops residue

The ready-gated wrapper's `kill -INT` went to the `ros2 run` parent, not the node
child; two nodes stayed up until they were found ~8-10 min later by PID. After
`SIGKILL` they remain **zombies** because the container's PID 1 is
`tail -f /dev/null` and never reaps; two `[perception_edge] <defunct>` entries
persist until the container is restarted (they consume no CPU). Next time kill
the actual `lib/jcheng_perception_edge/perception_edge` PID (or
`pkill -INT -f "[l]ib/jcheng_perception_edge/perception_edge"`) and check
`ps` afterwards. Pi load after stopping the edges was still ~7-10: the D435i
driver (~84 %), the 8765 gateway (~36 %) and Aurora (~30 %) dominate; the Pi
was responsive, no D-state or USB error.

## Caveats

- One static pose, one tag, no motion.
- The 3.67/3.07 mm cross-view figure is a consistency check against the same
  chain correction stored in `view_bias.yaml`, not an independent absolute
  truth; the per-view depth-vs-PnP bias (4.2 / 7.4 mm) and the tag curl still
  bound absolute accuracy.
- The tag sheet curls up to ~7 mm; flatten it before quoting absolute accuracy.
- The wrist depth stream was poor in run 3 (`aurora_wrist:depth_failures=200`)
  but enough tag observations matched; external depth is the stable one.
- The GPU external uplink reader wedge is a recurring ops issue; the reliable
  order here was edge first, then GPU restart. This needs a real fix (matching
  distros / reader recovery), not just a restart.
- The edge-side `velocity_min_dt_s` fix is not deployed to the Pi yet; the
  container still runs `eed1ca9`. A future bounded run should confirm the
  live tag track drops to the replay's ~8 mm or better.

## Files

- `tag_run3.json` — successful cross-view recording (observations, targets,
  status);
- `tag_run2.json` — wrist-only attempt (external reader wedge);
- `tag_wrist_fixed.csv`, `tag_ext_fixed.csv` — fixed-point check samples;
- `tag_report_raw.json`, `tag_report_corrected.json` — 5 mm gate before/after
  the chain correction;
- `replay_tag_run3.json`, `recorded_tag_run3.json` — fused drift (replay with
  `--source-bias none`, live recorded);
- `edge_tag_verify*.log`, `gpu_tag_verify*.log`, `record_tag*.log` — process
  logs;
- `ext_frame.png`, `wrist_frame.png` — the frames that show the tag in both
  views;
- `summary.json`, `hashes.txt`.

Reproduce the reports:

```bash
python3 scripts/fixed_point_report.py \
  --wrist evidence/hardware_20261124_tag_verify/tag_wrist_fixed.csv \
  --ext evidence/hardware_20261124_tag_verify/tag_ext_fixed.csv \
  --ext-offset 0.0099 -0.0018 0.0094 --limit 0.005 \
  --out evidence/hardware_20261124_tag_verify/tag_report_corrected.json
python3 scripts/replay_static_drift.py \
  --source evidence/hardware_20261124_tag_verify/tag_run3.json \
  --source-bias none --settle-s 6 --require reference_tag_0 --limit-mm 15 \
  --out evidence/hardware_20261124_tag_verify/replay_tag_run3.json
```

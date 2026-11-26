# Changelog

All notable changes to this project are documented here. Format based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); this project follows
[Semantic Versioning](https://semver.org/).
The `Unreleased` entries predate the documentation standard and are kept in
their original narrative order.

## [Unreleased]

- S3 predicted-mask relation evaluation (2026-11-26): on 39 synthetic scenes
  with the classical baseline, only 43.5% of GT instances match at IoU>=0.5;
  relation precision 0.850 but recall 0.035 / F1 0.067. The relation rule is
  not the bottleneck; segmentation and instance matching dominate. One new
  test (104 total). No runtime/DDS/device/motion change.
- S3 relation benchmark (2026-11-26): controlled 18-scene synthetic benchmark
  (6 contact / 6 near / 6 clear) with pair truth defined by construction gaps,
  evaluated on GT masks. `generate-relation-benchmark` and
  `evaluate-relations` clip into P/R/F1 1.0 for these controlled cases; this is
  a sanity check, not real-ore relation accuracy. One new test (103 total).
  No runtime, DDS, device or motion change.
- Scene State v2 relation graph prototype (2026-11-26):
  `build_relation_graph` emits evidence-backed `touching`/`near` edges (never
  `occludes` from a single RGB view); `build_scene_v2(..., relations=...)`
  validates references, self-edges and duplicates, and
  `compare_relation_sets` provides pair/predicate P/R/F1. Two new tests
  (102 total); independent relation labels are still missing, so no
  relation-accuracy claim.
- Scene State v2 geometry prototype (2026-11-26): `jcheng.scene_state.v2`
  schema, `build_scene_v2` and `validate_scene_v2` attach visible-only
  geometry to a frozen v1 record without changing v1. Missing geometry becomes
  explicit `not_computed`, and the validator rejects `true_centroid`,
  `full_extent`, `volume`, `mass` and grade/value fields. One new test
  (100 total); Scene Graph relations and multi-view identity remain planned.
  No runtime, DDS, device or motion change.
- Ore scene S2 visible geometry MVE (2026-11-26): new
  `jcheng_perception_scene/geometry.py` computes visible point count, depth
  valid fraction, visible centroid, 3x3 sample covariance, AABB and PCA OBB
  from an instance mask + metric depth + K + base<-camera transform. Invalid
  depth is never interpolated; empty masks/invalid depth return structured
  statuses. Two new tests (99 total) cover a 1 m plane, a base translation,
  zero depth, empty mask and shape mismatch. No real depth, no schema v2, no
  runtime/device/motion change; real-ore geometry remains Unverified.
- SAM2 tile converter (2026-11-26): new
  `jcheng_perception_scene/sam2_tiles.py` and `import-sam2-tiles` CLI convert
  exported SAM 2.1 tile masks (`result.json` + binary masks) into versioned
  scene-state JSONL without importing torch/SAM2. A YAML policy makes the tile
  size, area filter, merge threshold and per-image 504 override explicit.
  Four new tests (97 total) cover exact-partition replay, duplicate-mask
  skipping, config rejection and the CLI. On the independent 4-image split the
  converter reproduces AP50 0.6375 / AP50-95 0.3909 (P 0.649 / R 0.745,
  MAE 246.2). No runtime, DDS, device, depth or motion change.
- Ore scene understanding S1.5 / SAM 2.1 MVE (2026-11-26): off-line
  SAM 2.1 Hiera Tiny (Apache-2.0) on the remote AGX Orin, isolated from the
  SAM3/Qwen environment. Fixed 1008 tiles on 8 public held-out images:
  mask AP50 0.508 / AP50-95 0.328; a 504-px fallback for low-count images
  raises the aggregate to AP50 0.637 / AP50-95 0.387 (P 0.617 / R 0.775,
  count MAE 335.5) but costs ~2.5-4.8 min/image and over-segments. The
  fallback threshold and tiling parameters were fitted on those 8 images; a
  4-image independent split (selected before inference) reproduced AP50
  0.6375 / AP50-95 0.3909 (P 0.649 / R 0.745, MAE 246.3), but it is the same
  public dataset/annotation source, not an independent domain. A 2-image
  Rocks UAV scene-type check failed to transfer (AP50 0.301 / AP50-95 0.164,
  P 0.172, ~3.5x over-segmentation), so the tiling policy is not general.
  Real-ore accuracy is Unverified.
  No DDS, device, depth, motion, weight or `jcheng-interfaces` change.
  Evidence: `evidence/ore_scene_sam2_20261126/`; design/limits:
  `docs/ORE_SCENE_S1_5.md`.
- Ore scene understanding S0/S1 (2026-11-26): new ROS-free
  `jcheng_perception_scene` package (Ament Python) with a deterministic
  synthetic ore-pile generator, COCO polygon adapter, classical
  watershed/connected-component backends, versioned `jcheng.scene_state.v1`
  records, 2-D adjacency evidence, mask AP/PR/count/latency evaluation and a
  seven-command CLI (`scripts/ore_scene.py`). 16 new tests; 93 total pass;
  selftest covers empty scenes, truncated images and overlapping masks.
  Synthetic test (seed 20261127, 39 scenes / 600 GT): mask AP50 0.368,
  AP50-95 0.211, count MAE 6.51, latency P50 14.4 ms at 640x480. Public
  Mendeley `40_70mm` held-out (8 images / 9,208 GT, CC BY 4.0): AP50 0.043,
  count MAE 793.5 — reported as a domain-gap baseline, not an acceptance
  result. Real-ore accuracy is **Unverified**. No DDS, device, depth, motion,
  model weight or `jcheng-interfaces` change. Evidence:
  `evidence/ore_scene_s1_20261126/`; design: `docs/ORE_SCENE_*.md`,
  `docs/SCENE_REPRESENTATION.md`.
- Dataset and training pipeline for the real models:
  `scripts/capture_dataset.py` records the perception uplink JPEGs into session
  directories (manifest + provenance), `check_dataset.py` validates detect/pose
  labels, `split_dataset.py` splits whole sessions into train/val,
  `train_yolo.py` trains `yolov8n` detect or `yolov8n-pose` and can install
  `best.pt`, and `export_model.py` exports ONNX/OpenVINO. `docs/DATASET.md`
  defines the proposed five object classes and the two-contact-point wrist
  keypoint convention. An 8 s pilot capture (57 external + 51 wrist frames)
  validated the recorder; no labeled dataset or weights are committed.
- D435i depth-missing recovery (2026-11-25): a container restart can leave the
  depth unit closed (`uvcvideo -71`) while colour still streams, so the ingress
  exits on `depth=0` until the systemd start limit. Recovery: restart
  `armpi-rs-depth` (its IR warm-up retries), then `reset-failed` and start
  `jcheng-perception-ingress@d435i_ext`. Documented in
  `docs/FAILURE_MODES.md` §4.1; no motion.
- ADR 0004 executed on the robot (2026-11-24): `armpi-ext-cam.service` disabled;
  `armpi-rs-depth.service` now publishes colour plus aligned depth, and the
  config uses `/camera/aligned_depth_to_color/image_raw`. This retires the
  unaligned split UVC/depth topology.
- Split edge for DDS reader isolation: `perception_edge` gained
  `camera_filter`, `publish_targets`, `publish_status` and an ingress watchdog;
  new `perception_fusion` subscribes to the per-camera observation topics and
  publishes targets/status; `split_edge.launch.py` starts one ingress per
  camera plus fusion with respawn. Each ingress exits after 4 s without
  camera-frame/depth progress, so launch restarts a fresh DDS participant
  without touching the other view or fusion state. Static run 4: external 134 +
  wrist 226 continuous observations, cross-view 1.55 mm, fusion max 2.14 /
  p95 1.64 / final 1.96 mm; the external ingress watchdog restarted at 25 s and
  the fusion track survived. Core gained `max_track_speed_mps=0.5` /
  `max_target_distance_m=1.0` safety bounds. Deployment trap: colcon skipped
  reinstalling stale tracking/config by mtime; touch all core/edge sources and
  verify installed files. 77 tests.
- Repeatability check: with the motion gateway stopped and a fresh edge, run
  10 kept the external tag stream continuous for 60 s (267 observations) but
  the wrist depth reader wedged at 7.5 s (26 observations). The motion gateway
  was stopped at 20:57:22, the wrist stopped around 20:57:51 and the gateway
  restarted at 20:58:34, so it was not the trigger. Fused max/p95/final was
  6.36/2.53/0.27 mm and cross-view 2.77 mm while the wrist was present, but the
  full dual-view window is **not repeatable** yet. The remaining blocker is the
  edge wrist-depth DDS reader; the next step is per-camera ingress/participant
  isolation (the aux-gateway CycloneDDS fix stays).
- Gateway root cause and dual-view tag confirmation. The read-only aux gateway
  (8765) did not pass `RMW_IMPLEMENTATION`/`CYCLONEDDS_URI`, so it subscribed to
  the external colour topic with Fast DDS while the camera and edge use
  CycloneDDS; large image samples across RMWs wedged the edge's external RGB
  reader. The run script and `/etc/armpi-gateway/gateway.env` now force
  CycloneDDS (backups in `/home/pi/armpi-gateway-rwm-20261124/`). The 8766
  motion gateway was an orphan PPID-0 process still consuming the wrist
  pointcloud and blocking Aurora depth; killing it recovered the wrist depth
  reader. Run 9 then delivered the **dual-view** result: 240 external + 195
  wrist tag observations over 60 s, cross-view **3.37 mm**, fused track p95
  **3.78 mm**, final **1.19 mm**, max 10.03 mm (a 12.4 mm view-switch transient
  at t=26.2 s), no 67 mm reference step. The motion gateway was stopped for
  run 9 but its service was active again at the final check; it needs its
  pointcloud subscription/QoS changed or must be stopped during perception.
- Deploy image for the GPU node: `deploy/Dockerfile.gpu` builds a ROS 2 Humble
  image with `/opt/jcheng/venv` (torch 2.9.1+cu128, ultralytics 8.4.216,
  onnxruntime 1.32.0, openvino 2026.4.1, numpy 1.26.4) and the apt
  `python3-opencv` 4.5.4 build that carries ArUco. It deliberately does not
  install `opencv-python`, whose wheel would shadow the ArUco-capable system
  cv2. Not built or GPU-verified yet: the host has no nvidia-container
  runtime, so `--gpus all` and the one-time sudo install are documented in
  `deploy/README.md`.
- Diagnostic: recreating a camera's image/depth/CameraInfo subscriptions after
  a 4 s stall did **not** recover the wedged readers. In the test the external
  RGB reader stayed at 5 frames while its depth advanced, and the wrist depth
  reader stayed at 7 while its RGB advanced; three recreations each did not
  help. The wedge is at the DDS participant/reader level, so the watchdog was
  reverted; the Pi runs the committed uplink clock-jump/worker fix only. The
  next step is reader isolation (separate participants/relay) or a
  CycloneDDS version/configuration fix.
- Uplink robustness and a same-distro Humble run. The per-camera uplink rate
  limiter now resets its baseline after a backwards clock jump
  (`uplink_allowed`, 3 new tests), and `UplinkWorker` catches encode/publish
  exceptions so one transient DDS failure cannot silently kill a camera's
  uplink thread; status notes expose `encode_failures`/`rate_limited`. With the
  laptop GPU node running inside the existing `armpi-humble-moveit` image
  (`jcheng-humble-tag`: host network, wired CycloneDDS, `python3-opencv` with
  ArUco, `/workspace/tag_ws` built in Humble), a 60 s tag run kept the edge at
  468 frames and produced a **wrist-only** fused track max 7.30 mm / p95
  4.66 mm — meeting the ~8 mm target numerically, but external tag
  observations were still 0 (external GPU frames stopped at 42 and a 12 s
  check saw no external uplink), so the dual-view confirmation is pending.
  74 tests.
- Reference-point (AprilTag) mode: new GPU `aruco` backend and
  `config/gpu_tag.yaml`. A tag detection carries its centre as the primary
  keypoint (corners follow at lower confidence), so the existing edge keypoint
  path back-projects one physical point instead of the object-dependent bbox
  centre. This removes the 57-67 mm cup difference at the source and turns the
  first cross-view pair step into a millimetre-scale residual (new fusion test
  with a 3 mm residual stays under 5 mm). `scripts/fixed_point_check.py`
  remains the absolute depth/PnP reference; the printed tag curl and the ~4 mm
  depth-vs-PnP bias still bound absolute accuracy. Hardware validation on
  2026-11-24: two-view tag-centre difference 3.77 mm, fixed-point 5 mm gate
  3.67 mm (depth) / 3.07 mm (PnP) after `view_bias.yaml` (raw 15.92/12.79 mm
  FAIL), and no 67 mm first-pair step. The tag run also exposed near-duplicate
  capture stamps; see the velocity min-baseline entry below.
- Velocity min-baseline: `velocity_min_dt_s = 0.10` rejects same-camera
  differences closer than 100 ms. A tag run had 47/191 wrist observations with
  capture dt <= 0; a 1 mm difference over 0.04 s fabricated ~25 mm/s and a
  10 mm extrapolation. The fix makes the tag fused replay 8.3 mm (was 10.7)
  and restores the offline/live agreement for the earlier hardware runs
  (4.63/7.91 mm replay vs 4.63/8.13 mm live). 71 tests. Deployed in `f90b5d5`;
  two external-only tag reruns confirmed 0.4 mm and 1.8 mm after settle, and an
  Aurora driver restart cut wrist depth failures from 365 to 1, but the edge
  intake stalled after ~6 s under Pi load (88 frames), so a dual-view rerun is
  still pending.
- Cross-view bias model and static-track drift fix. The observation model is
  `z_i = x + b_i + m_i + n_i`: the wrist view is the reference and only the
  external relative offset is observable. New `config/view_bias.yaml` carries
  the chain-level correction (+9.9, -1.8, +9.4 mm, 6 mm 1 sigma) from the run-3
  tag and table plane; `Track.view_offset` learns the object-dependent
  bbox-centre residual online from time-matched pairs, including cross-batch
  pairs. Velocity is now only formed from two successive samples of the same
  camera (`update_velocity`, `H = [0|I]`); position updates use
  `velocity_gain = 0`, the initial velocity sigma is 0.25 m/s (was 1.0) and
  publishing/association extrapolation is capped at 0.35 s. Offline replay of
  run-7 observations: worst static-target drift 696 mm recorded / 478 mm with
  the old tracker -> 6.4 mm (cup 4.1, table 4.9, chair 6.4, person 4.4). New
  `scripts/replay_static_drift.py` and 7 tests (65 total). Deployed to the Pi 5
  as `eed1ca9` and statically validated on 2026-11-24: two bounded runs (90 s /
  60 s, no motion) gave worst required-label drift 4.7 mm / 8.1 mm versus
  448-696 mm pre-fix; the first cross-view pair causes a one-time 67 mm
  reference step (documented), run 2 had poor wrist depth matching, and
  absolute position is still a per-view bbox-centre point until a defined
  point/keypoint is used. Evidence: `evidence/hardware_20261124_drift_verify/`.
- Cross-machine real YOLO detection path: installed `rmw_cyclonedds_cpp` on the
  Jazzy laptop and ran the external camera through the Pi 5 edge to a laptop
  RTX 3060 (`yolov8s.pt`). GPU 2.06 fps / 55.2 ms average, 37 frames received,
  0 failed; edge saw 16 detections with the original capture stamp. All were
  dropped at depth matching because the external depth is still the unaligned
  split stream (1280x720 colour vs 640x480 depth). Detection path only; no
  metric observations or motion.
- Clean shutdown: the edge, GPU and synthetic nodes ignore a repeated SIGINT
  during teardown, so `timeout -s INT` no longer prints a `KeyboardInterrupt`
  traceback. The synthetic demo exits with all processes reporting cleanly.
- Measured wrist TF: the invalid z was traced to the vendor `/joint_states`
  being a command cache (joint1 +0.838, joints 2-5 zero) while the bus servos
  read joint1 -0.013, joint2 +0.406, joint3 -1.382, joint4 -1.629 rad. New
  `wrist_tf` polls `/ros_robot_controller/bus_servo/get_state` at 10 Hz,
  parses the live `/robot_description`, composes the validated hand-eye and
  publishes `base_link -> camera_optical_measured`; `cameras.yaml` now uses
  that frame. Pi 5 validation: 1000+ reads, 0 failures; table-plane fit
  z=-0.0836 m vs the -0.0825 m reference (1.1 mm), tilt 1.21 deg, RMS 0.61 mm.
  Single static pose only; not cleared for motion.
- Wrist kinematics (URDF chain/FK, pulse conversion) and config tests — 58
  tests total.
- Pi 5 ARM64 build (4 packages, 27.1 s) and synthetic end-to-end (6.2/6.3 Hz,
  match 0.994, 48 ms, one fused track).
- Wrist Aurora driver rebuilt from the local aarch64 SDK; real RGB/depth stream
  restored (6.7/3.1–6.4 Hz). A real wrist data-path test with a mock blue
  detector emitted `blue_block-001` (FINE, wrist weight 1.0, GPU 0 failures);
  the metric z was invalid at the time (root cause fixed by the measured wrist
  TF entry above), so the result was not used for motion.
- CameraInfo subscription switched to sensor QoS: the vendor Aurora publisher is
  best-effort/volatile, which was incompatible with the previous latched QoS.
- External D435i remains the split UVC-colour + depth-only topology; aligned
  depth is still pending ADR 0004.
- Shadow detector launch/config: a second GPU node reads the same uplink and
  publishes to `/perception/*_shadow`, with its own status topic. Production
  fusion is unaffected.
- Detector backends: Ultralytics, ONNX Runtime (CPU), OpenVINO IR (CPU), OpenCV
  DNN (legacy models) and mock. ONNX Runtime, OpenVINO and Ultralytics CPU were
  installed in an isolated venv and verified on the vendor models:
  `yolov8s.onnx` 197.5 ms, `yolov8s.xml` 126.7 ms, `yolov8s.pt` 1.04 s first
  frame (4 detections each); OBB `yolov8n-obb.pt` also passed.
- `python_executable` launch argument and `scripts/setup_gpu_venv.sh` for
  running the ROS node with a venv runtime; ONNX Runtime and Ultralytics were
  verified in-node (232.8 ms and 154.3 ms steady per frame on CPU).
- CUDA PyTorch installed and verified: `torch 2.9.1+cu128`,
  `torchvision 0.24.1+cu128`, RTX 3060; `yolov8s.pt` 9.0 ms FP32 / 7.4 ms FP16 steady per image,
  ROS node 54.3 ms/frame average. A torch 2.11/NCCL mismatch was resolved by
  pinning the 2.9.1 pair; `RUNTIME=ultralytics-cuda` reproduces it.
- `scripts/check_gpu_backend.py`: ROS-free detector smoke check for every
  backend; reports missing runtime/model explicitly.
- Detector adapter tests (mock, unknown backend, missing model) — 58 tests total.
- ADR 0004: proposed single RealSense owner with aligned depth; documents why
  the currently deployed split UVC-colour + depth-only topology cannot be used
  for metric fusion.

## [0.1.0] — 2026-11-23

Initial distributed-perception implementation.

- Four ROS 2 packages: `jcheng_perception_msgs`, `jcheng_perception_core`,
  `jcheng_perception_edge`, `jcheng_perception_gpu`.
- Pi 5 edge: JPEG uplink with capture timestamps, per-camera depth and TF
  timestamp rings, capture-time matching, depth back-projection, association,
  distance-weighted two-view fusion, constant-velocity Kalman tracking.
- Laptop GPU: asynchronous two-stream YOLO with bounded per-camera queues,
  hot reload, keypoint support and capture-timestamp preservation.
- Measured timings on an x86 Jazzy host: edge callbacks < 1 ms, 10 Hz per
  camera, 99% time-match success, one fused track with both views.
- 53 ROS-free unit tests; four-package colcon build; synthetic end-to-end demo.
- No hardware deployment and no real YOLO model in this repository.

## Version policy

`major` changes a topic/message contract incompatibly; `minor` adds a
capability or camera; `patch` fixes a defect or documentation. The message
package is provisional until promoted to `jcheng-interfaces`.

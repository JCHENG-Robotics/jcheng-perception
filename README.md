# JCHENG Perception

**English** | [简体中文](README.zh-CN.md)

**Distributed robot perception (design D).** The Raspberry Pi 5 acquires images,
decodes depth, maintains TF, keeps timestamp-indexed rings, back-projects 3D
points, associates targets and runs the Kalman fusion; the laptop GPU runs two
asynchronous YOLO detectors. Images cross ROS 2 compressed, detections return
with the **capture timestamp**, the Pi 5 matches depth and TF by that timestamp,
weights the two views by distance-dependent variance, and publishes one fused
target pose for the arm planner.

> Status: **Experimental** — v0.1.0 as of 2026-11-25: code, offline simulation,
> single-host synthetic, Pi 5 ARM64 synthetic, the wrist real-camera data path,
> bus-measured wrist TF, the single static-pose table-plane check, the
> cross-machine YOLO detection path, the ADR 0004 aligned external camera, the
> split-edge ingress/fusion pipeline and a single 60 s dual-view tag run are
> verified. Multi-pose accuracy, a repeatable full dual-view window and motion
> integration are not.
> This repository contains no model weights, no vendor code and publishes no
> motion command.
> **Ore scene understanding (offline S1, 2026-11-26):** a ROS-free
> `jcheng_perception_scene` submodule adds instance masks, a versioned scene
> state, a reproducible CLI and mask-AP evaluation. Synthetic pipeline metrics
> are verified; real-ore accuracy is **Unverified**. No DDS, device, depth or
> motion path is added. See [docs/ORE_SCENE_S1.md](docs/ORE_SCENE_S1.md).
> English canonical; the Chinese README is a translation.

---

## 1. Why this split

| Constraint | Design response |
|---|---|
| Pi 5 compute is limited; YOLO would block acquisition/control | All YOLO runs on the laptop GPU; the Pi 5 does millisecond-scale geometry and filtering |
| The two cameras run at different rates (Aurora 12 Hz RGB / 8 Hz depth) | Every message keeps its capture timestamp; the Pi 5 looks up the depth and TF rings by timestamp |
| Wrist-camera TF and image must align while moving | Depth frames use the TF at their own timestamp; the ring interpolates linearly + slerp |
| External and wrist cameras have complementary accuracy | Information weighting by covariance, range, confidence and occlusion |
| Occlusion tolerance | The other view keeps tracking when one fails; the track coasts briefly and is then lost |
| Model iteration must not disturb control | GPU hot reload, one model per camera, bounded async queues, best-effort status streams |
| Startup transients produce duplicate tracks | Cross-camera duplicate merge (same label, distance gate, strict same-camera gate) |

## 2. Data flow

```text
        D435i (USB3, external)               Aurora 930 (wrist, USB2)
               │                                      │
               └──────────────┬───────────────────────┘
                              ▼
                 ┌──────────────────────────────┐
                 │  Pi 5 · jcheng_perception_edge│
                 │  · RGB acquisition / JPEG     │
                 │  · depth ring + TF ring       │
                 │  · 3D back-projection / association / weighted fusion │
                 │  · Kalman prediction / 30 Hz output │
                 └───────┬───────────────▲───────┘
       CompressedImage   │               │  Detection2DArray
       + capture stamp   ▼               │  + original capture stamp
                 ┌──────────────────────────────┐
                 │ Laptop GPU · jcheng_perception_gpu │
                 │  · per-camera async queue/thread │
                 │  · YOLO (Ultralytics/OpenVINO) │
                 │  · keypoints / hot reload / latency stats │
                 └──────────────────────────────┘
                              │
                              ▼
                 /perception/targets  FusedTargetArray
                 → grasp planning / drop recheck
```

**Key contract: the GPU node never rewrites `header.stamp`.** It returns the
Pi 5 capture time unchanged. When timestamp matching fails the observation is
dropped; a "latest frame" is never substituted.

## 3. Camera roles

| Camera | Role | Main duty | Typical range |
|---|---|---|---|
| Chassis-front RealSense D435i | `external` | coarse localisation, target search, drop recheck | 0.15–2.0 m |
| Wrist Aurora 930 | `wrist` | fine grasp keypoints, close-range pose | 0.10–0.90 m |

Task phases: `SEARCH → COARSE → ACQUIRE → FINE → DROP_RECHECK`. The external
camera dominates in `COARSE`/`DROP_RECHECK`, the wrist camera in `FINE`; the
phase prior only shifts relative information weight — absolute accuracy is
still set by the range-dependent depth variance.

## 4. Repository layout

```text
jcheng-perception/
├── config/
│   ├── cameras.yaml              # real cameras/topics/noise/range
│   ├── cameras_synthetic.yaml    # synthetic demo
│   ├── wrist_tf.yaml             # bus-measured wrist TF / hand-eye / servo mapping
│   ├── fusion.yaml               # association / filtering / phase priors
│   ├── fusion_demo.yaml
│   ├── frames.yaml               # external static extrinsic fallback
│   ├── gpu.yaml                  # real GPU model path (Ultralytics)
│   ├── gpu_mock.yaml             # model-free demo
│   ├── gpu_onnx.example.yaml     # ONNX Runtime CPU
│   ├── gpu_openvino.example.yaml # OpenVINO IR CPU
│   ├── gpu_tag.yaml              # AprilTag reference-point mode
│   ├── gpu_shadow.example.yaml   # shadow-model template
│   ├── ore_scene.yaml            # offline ore S1 (synthetic defaults)
│   ├── ore_scene_public_40_70mm.yaml  # offline ore S1 (public 5 MP profile)
│   └── sam2_tiles.example.yaml   # SAM2 tile merge / per-image override policy
├── docs/
│   ├── ARCHITECTURE.md           # node roles, processes/threads, data flow
│   ├── TOPICS.md                 # topics/QoS/field semantics
│   ├── TIMESTAMP_AND_TF.md       # timestamp and TF contract
│   ├── FUSION.md                 # weighting, phases, filtering
│   ├── DEPLOYMENT.md             # Pi 5 + laptop deployment
│   ├── MODEL_ITERATION.md        # model iteration / hot reload / debugging
│   ├── CALIBRATION.md            # reuse / update calibration
│   ├── FAILURE_MODES.md          # faults and degradation
│   ├── EVIDENCE.md               # verification record for this version
│   ├── ORE_SCENE_RESEARCH.md     # ore scene research (Chinese)
│   ├── ORE_SCENE_UNDERSTANDING.md# architecture/module boundaries (Chinese)
│   ├── ORE_SCENE_ROADMAP.md      # S0-S8 roadmap (Chinese)
│   ├── SCENE_REPRESENTATION.md   # scene state v1 design (Chinese)
│   ├── ORE_SCENE_EVALUATION.md   # evaluation protocol (Chinese)
│   ├── ORE_SCENE_S1.md           # S1 validation report (Chinese)
│   ├── ORE_SCENE_S1_5.md         # SAM 2.1 offline MVE report (Chinese)
│   └── adr/
├── ros2_ws/src/
│   ├── jcheng_perception_msgs/   # provisional IDL (M2 promotes to jcheng-interfaces)
│   ├── jcheng_perception_core/   # pure Python, no ROS dependency
│   ├── jcheng_perception_edge/   # Pi 5 node + synthetic scene + wrist_tf
│   ├── jcheng_perception_gpu/    # laptop GPU detector node
│   └── jcheng_perception_scene/  # offline ore S1 (ROS-free, no weights)
├── scripts/
│   ├── simulate_pipeline.py      # offline full-pipeline simulation
│   ├── ore_scene.py              # offline ore S1 CLI
│   ├── run_core_tests.sh
│   └── check_repo.sh
├── tests/                        # 97 ROS-free tests (77 existing + 20 ore scene)
└── capability.yaml
```

## 5. Quick start

### 5.1 Core only (no ROS, no hardware)

```bash
python3 -m pytest tests -q
python3 scripts/simulate_pipeline.py
```

The simulation covers a moving target, an external occlusion window, GPU
latency, TF interpolation, two-view fusion and `DROP_RECHECK`.

### 5.2 Build the ROS 2 packages

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash      # robot; a Jazzy host also builds
colcon build --packages-select \
  jcheng_perception_msgs jcheng_perception_core \
  jcheng_perception_edge jcheng_perception_gpu \
  jcheng_perception_scene          # offline ore S1 only; omit for runtime-only builds
source install/setup.bash
```

### 5.3 Single-host synthetic demo (no model, no GPU)

```bash
export ROS_DOMAIN_ID=77                # isolate from stale DDS participants
ros2 launch jcheng_perception_gpu synthetic_demo.launch.py
```

Expected: **one** two-view track on `/perception/targets`, `phase=3` (FINE),
weights around `external 0.2 / wrist 0.8`, and `match_success_rate≈1.0` on
`/perception/status`.

### 5.4 Real deployment

```bash
# Pi 5 (robot container/host; source the vendor workspace first for
# ros_robot_controller_msgs and the bus service)
ros2 launch jcheng_perception_edge pi5_edge.launch.py
# this launch also starts wrist_tf by default; it can run alone:
# ros2 launch jcheng_perception_edge wrist_tf.launch.py

# Laptop GPU
ros2 launch jcheng_perception_gpu laptop_gpu.launch.py
```

Deployment, DDS, camera drivers and single-device ownership conflicts are in
[DEPLOYMENT.md](docs/DEPLOYMENT.md). The real wrist TF must come from
`wrist_tf`'s bus-measured joints, not the vendor `/joint_states` command cache;
a wrist observation is dropped when `camera_optical_measured` is missing.

## 6. Output contract

| Topic | Type | QoS | Semantics |
|---|---|---|---|
| `/perception/uplink/<cam>/image_compressed` | `sensor_msgs/CompressedImage` | best effort, depth 1 | JPEG; `header.stamp` = capture time |
| `/perception/detections/<cam>` | `Detection2DArray` | best effort, depth 10 | image-frame boxes/keypoints; stamp returned unchanged |
| `/perception/observations/<cam>` | `TargetObservation` | best effort, depth 10 | single-camera 3D observation (debug/evidence) |
| `/perception/targets` | `FusedTargetArray` | best effort, depth 10 | fused target; 30 Hz predicted output |
| `/perception/status` | `PerceptionStatus` | best effort, depth 5 | match rate, latency, phase, track count |
| `/perception/gpu_status` | `GpuStatus` | best effort, depth 5 | GPU queue/inference/model version |
| `~/drop_recheck`, `~/reset` | `std_srvs/Trigger` | service | force drop recheck / clear tracks |

Full fields: [TOPICS.md](docs/TOPICS.md).

## 7. Measured behaviour (synthetic, 2026-11-23)

| Metric | Measured |
|---|---|
| Per-camera uplink/detection/observation | 10 Hz / 10 Hz / 10 Hz |
| Timestamp match rate | 99.0–100% |
| Detection capture → edge callback | 27–40 ms (same host, mock detector) |
| Edge callback cost | image 0.4 ms, depth 0.1 ms, detections 0.6 ms, publish 1.3 ms, tf 0.3 ms |
| Output track | 1 confirmed, two-view fused, weights 0.21 / 0.79, phase FINE |
| Real-model CPU smoke | ONNX 197.5 ms / OpenVINO 126.7 ms / Ultralytics 1.04 s (first frame), 4 boxes each |
| Real-model GPU smoke | RTX 3060: Ultralytics steady **9.0 ms (FP32) / 7.4 ms (FP16)** per image; ROS node 54.3 ms/frame average |
| Unit tests | 77 passed (current `main`) |
| colcon build | 4 packages (Jazzy host) |

> These synthetic numbers are not hardware accuracy or latency validation. The
> real-model smoke test is **single-image inference**, not Pi 5/camera/network
> end-to-end performance.

> An isolated venv has **ONNX Runtime 1.32.0**, **OpenVINO 2026.4.1**,
> **PyTorch 2.9.1+cu128 / 2.6.0+cpu, torchvision 0.24.1+cu128 / 0.21.0+cpu,
> Ultralytics 8.4.216**; the CUDA build uses the RTX 3060 and
> `torch.cuda.is_available()=True`. The ROS node verified ONNX Runtime,
> Ultralytics CPU and CUDA through the venv Python.

### 7.1 Pi 5 and real cameras (data path + TF + aligned depth + dual-view)

| Item | Result |
|---|---|
| Pi 5 ARM64 build | 4 packages, 27.1 s |
| Pi 5 synthetic end-to-end | 6.2/6.3 Hz, match 99.4%, 48 ms latency, one two-view track |
| Wrist Aurora | driver rebuilt from the local SDK; RGB 6.7–6.8 Hz, depth 3.1–6.4 Hz, 640×400 |
| Wrist real data path (mock blue detector) | GPU 4.9 Hz / 0 failures; edge emitted `blue_block-001`, FINE, wrist weight 1.0 |
| Wrist metric TF | `wrist_tf` bus-measured at 10 Hz, 1000+ reads / 0 failures; `camera_optical_measured` |
| Table-plane check | z=-0.0836 m (reference -0.0825 m, difference 1.1 mm), tilt 1.21°, RMS 0.61 mm; **single static pose** |
| Cross-machine real YOLO (external) | laptop RTX 3060 + CycloneDDS; GPU 2.06 fps / 55 ms, edge received 16 detections; capture stamp returned unchanged (run on the earlier split camera topology) |
| External camera (ADR 0004) | Executed 2026-11-24: `armpi-ext-cam.service` disabled; `armpi-rs-depth.service` owns the D435i and publishes colour + aligned depth; config uses `/camera/aligned_depth_to_color/image_raw` |
| Aligned-depth rate | Colour ~6.5 Hz / aligned depth ~3.9 Hz on the rebuilt 2026-11-25 image (`jcheng-system` deploy acceptance) |
| Split edge | Per-camera ingress + fusion with respawn; static run 4: 134 external + 226 wrist observations, cross-view 1.55 mm, fusion max/p95/final 2.14 / 1.64 / 1.96 mm; the external ingress watchdog restarted at 25 s and the fused track survived |
| Dual-view tag run 9 | 240 external + 195 wrist observations over 60 s; cross-view 3.37 mm; fused p95 3.78 mm, final 1.19 mm, max 10.03 mm (12.4 mm view-switch transient); no 67 mm reference step |
| Repeatability run 10 | External stream continuous for 60 s (267 observations); the wrist depth reader wedged at 7.5 s (26 observations); the full dual-view window is **not repeatable** |
| D435i depth recovery | If depth is missing (`uvcvideo -71`), restart `armpi-rs-depth`, then `reset-failed`/start the ingress; documented in [FAILURE_MODES.md](docs/FAILURE_MODES.md) |

> The dual-view result is a single 60 s run; run 10 shows the wrist depth DDS
> reader can still wedge, so the full dual-view window is not yet repeatable.
> The aligned-depth topology is deployed; multi-pose/fixed-target accuracy and
> TF lag during motion remain unverified. The current poses must not be used
> for motion. The test sent no motion command.

## 8. Design boundaries

- **No motion control**: publishes no `cmd_vel`, joint, gripper, serial or CAN
  command; the output is an advisory pose.
- **No safety decision**: the planner/safety layer must validate workspace,
  limits and target freshness itself.
- **No YOLO on the Pi 5**: `jcheng_perception_edge`/`core` import no detector
  (enforced by a static test).
- **No fabricated depth/TF**: a failed time match is dropped; the wrist camera
  has no static extrinsic fallback.
- **No vendor `/joint_states` for wrist geometry**: it is a command cache;
  `wrist_tf` publishes `camera_optical_measured` from bus-measured joints +
  URDF FK + hand-eye, and publishes nothing when the bus read fails.
- **No model weights in the repository**: see
  [MODEL_ITERATION.md](docs/MODEL_ITERATION.md) and
  [LICENSE-REVIEW.md](LICENSE-REVIEW.md).

## 9. References and sources

- **Robot structure and calibration**: `jcheng-manipulation` v1.0.0
  - TF tree and `camera_optical_calibrated`:
    [`docs/hardware/tf_tree.md`](https://github.com/JerichoCheng/jcheng-manipulation/blob/main/docs/hardware/tf_tree.md)
  - Wrist hand-eye and optical frame:
    [`docs/hardware/camera_frames.md`](https://github.com/JerichoCheng/jcheng-manipulation/blob/main/docs/hardware/camera_frames.md)
  - Chassis D435i intrinsics/extrinsics (2026-11-20):
    `evidence/calibration/20261120/`
- **YOLO reference**: the vendor `ArmPiUltra` `yolov8_detect` / `yolo_detect`
  (Ultralytics + OpenVINO, `ObjectInfo` publishing). This repository keeps
  "one model per camera + Ultralytics/OpenVINO export" but emits a
  capture-timestamped, ROS-decoupled `Detection2DArray`.
- **`lerobot_piper_local`**: the local copy contains acquisition/camera patches
  and **no YOLO code**; this repository borrows the "configuration-first,
  health-check-first, fail-early" engineering habit and the serial-number/keyed
  camera management. Use the vendor `yolov8_detect` as the YOLO reference.

## 10. Documentation

- [Architecture and threading](docs/ARCHITECTURE.md)
- [Topics and QoS](docs/TOPICS.md)
- [Timestamp and TF contract](docs/TIMESTAMP_AND_TF.md)
- [Fusion and filtering](docs/FUSION.md)
- [Deployment](docs/DEPLOYMENT.md)
- [Model iteration](docs/MODEL_ITERATION.md)
- [Calibration](docs/CALIBRATION.md)
- [Failure modes](docs/FAILURE_MODES.md)
- [Hardware bring-up record](docs/HARDWARE_BRINGUP.md)
- [Verification evidence](docs/EVIDENCE.md)
- [Acceptance report](ACCEPTANCE.md)
- ADR: [edge/GPU split](docs/adr/0001-edge-gpu-split.md) ·
  [capture-timestamp contract](docs/adr/0002-capture-timestamp-contract.md) ·
  [distance-weighted fusion](docs/adr/0003-distance-weighted-fusion.md) ·
  [D435i single owner](docs/adr/0004-camera-device-ownership.md)

## 11. License

The license review is in progress; this repository has no `LICENSE` and all
rights are reserved. Model weights and third-party inference runtimes remain
under their own licenses; see [LICENSE-REVIEW.md](LICENSE-REVIEW.md).

## 12. Ore scene understanding (offline S1)

The `jcheng_perception_scene` package is a **ROS-free, offline** submodule for
ore/rock instance segmentation, versioned scene state and evidence-labelled
evaluation. It does not join DDS, open devices, read depth or publish motion.

```bash
# Deterministic synthetic ore pile (640x480, 36 scenes + 3 empty)
python3 scripts/ore_scene.py generate-synthetic --out /tmp/ore-data \
  --scenes 36 --empty-scenes 3 --seed 20261127
python3 scripts/ore_scene.py run --dataset /tmp/ore-data --out /tmp/ore-run \
  --config config/ore_scene.yaml
python3 scripts/ore_scene.py selftest --out /tmp/ore-selftest

# SAM2 tile export -> scene state (offline; no torch/SAM2 import)
python3 scripts/ore_scene.py import-sam2-tiles \
  --dataset /tmp/ore-data --out /tmp/ore-sam2 \
  --tiles config/sam2_tiles.example.yaml --metrics
```

S1 results (2026-11-26, CPU only):

| Data | Scope | mask AP50 | mask AP50-95 | Count MAE |
|---|---|---|---|---|
| synthetic, seed 20261127 | 39 scenes / 600 GT | 0.368 | 0.211 | 6.51 |
| public Mendeley `40_70mm`, held-out | 8 images / 9,208 GT | 0.043 | 0.012 | 793.5 |

The synthetic result verifies the pipeline, not real-ore accuracy. The public
result is a **domain-gap baseline** (the classical baseline under-segments dense
real piles) and is not an acceptance result. No real-ore accuracy is claimed.

**S1.5 (2026-11-26, offline):** SAM 2.1 Hiera Tiny (Apache-2.0) on the remote
AGX Orin. On 8 public held-out images, fixed 1008 tiles reach mask AP50 0.508 /
AP50-95 0.328; a 504-px fallback for low-count images raises this to AP50 0.637
/ AP50-95 0.387, but the threshold/merge parameters were fitted on those 8
images and over-segment. A 4-image independent split (selected before inference)
reproduced AP50 0.6375 / AP50-95 0.3909; it is the same public dataset, not an
independent domain. A 2-image Rocks UAV scene-type check failed to transfer
(AP50 0.301, precision 0.172, ~3.5x over-segmentation), so the tiling policy is
not general. Real-ore accuracy remains Unverified.

Design and evidence:

- [S1 validation report](docs/ORE_SCENE_S1.md)
- [S1.5 SAM 2.1 offline MVE](docs/ORE_SCENE_S1_5.md)
- [Research](docs/ORE_SCENE_RESEARCH.md) ·
  [Architecture](docs/ORE_SCENE_UNDERSTANDING.md) ·
  [Roadmap](docs/ORE_SCENE_ROADMAP.md) ·
  [Scene representation](docs/SCENE_REPRESENTATION.md) ·
  [Evaluation](docs/ORE_SCENE_EVALUATION.md)
- [Evidence bundle](evidence/ore_scene_s1_20261126/README.md)

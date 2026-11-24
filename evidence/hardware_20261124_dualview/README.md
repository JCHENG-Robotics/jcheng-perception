# 2026-11-24 — dual-camera real-YOLO two-view fusion (first hardware proof)

Robot: ArmPi Ultra, Pi 5 (ROS 2 Humble, container `ArmPiUltra`) + laptop RTX
3060 (ROS 2 Jazzy). No motion commands. Arm stationary at camera
(0.181, 0.010, 0.192) m. A wireless computer mouse was placed at
`base_link ≈ (0.40, +0.02, table)`; both cameras see it.

## What was proven

At t = 8.21 s one coalesced detection batch carried both views of the same
object and produced a **two-view fused target**:

```text
external observation   mouse 0.89  (0.36222, -0.01639, -0.06985) m  depth 0.246 m  w 0.67
wrist observation      mouse 0.95  (0.38551, -0.02056, -0.05310) m  depth 0.315 m  w 0.33
FusedTarget mouse-001           (0.34177, -0.01369, -0.08519) m
  weight_external 0.67  weight_wrist 0.33
  source_cameras ['d435i_ext', 'aurora_wrist']  depths (0.246, 0.315)
  phase COARSE, status CONFIRMED, confidence 0.90
```

13 published samples (0.3 s) carried both weights and both source cameras.
The fusion path, per-view weights and `source_cameras` work on hardware.

The raw views agree to `(23.4, 4.2, 16.8) mm` (29 mm). That is **not** a new
TF error: the mouse is a 3-D object, the external bbox centre lands on its
side/front, the wrist bbox centre on its top, and the external chain carries
its known ~11 mm table-plane offset (see `hardware_20261124_plane_check/`).
A defined point (AprilTag) is still the right accuracy target.

## Why this took four runs

1. **Coalesce window (design finding).** The production `coalesce_ms: 30`
   groups detection batches by *receive* time. The two cameras are captured
   asynchronously (2/4 fps uplink) and the GPU returns them with different
   latency (46–320 ms), so 30 ms never pairs them and every update is
   single-view. This run used a test `fusion_config` with
   `coalesce_ms: 1000`; six of the 46 mouse observations were still
   single-view. Fix direction: coalesce by capture-time proximity or keep a
   short per-camera observation buffer, not by GPU arrival time.
2. **Stale fallback intrinsics.** `cameras_fixed_target.yaml` carried the UVC
   1280×720 values (`fx 892`, `cx 648`). Before the driver `CameraInfo`
   arrived the edge back-projected 640×480 pixels with them: the first
   external mouse observation was `(0.381, +0.083, -0.045)` — 19/99/25 mm off
   the correct `(0.362, -0.016, -0.070)`. The test config now carries the
   D435i 640×480 values (`fx 606.50/fy 606.60/cx 325.03/cy 249.19`, D = 0).
   **`config/cameras.yaml` in the repo still has the UVC values and needs the
   same fix.**
3. **GPU reader wedge.** After several edge restarts the GPU node's
   *external-only* subscription stopped delivering (`frames_received` froze at
   47) while a fresh laptop subscriber received 17 uplinks from the same
   topic in 13 s. Restarting the GPU node restored both streams
   (`frames_received` 0→42 external, 0→51 wrist in this run). Cross-distro
   Humble↔Jazzy CycloneDDS again; needs an ops rule (restart the GPU node per
   run) and a real fix (matching distros / a bridge).
4. **Two-view track state drift.** The fused target's published position
   `(0.3418, -0.0137, -0.0852)` is outside the two observations, and that
   track later drifted to `(0.555, -0.056, +0.069)`. A static mouse must not
   move. The KF/table logic reacting to the 29 mm inter-view offset is the
   next accuracy bug; do not quote fused absolute positions until it is
   fixed.

## Rates and health (32 s run)

```text
edge   camera_fps 3.2–3.7 / 2.5–2.9   frames 166   detections 212   matched 134
GPU    ~1.4 fps per camera            inference 46–50 ms after warm-up
       frames_received 42 external / 51 wrist, 0 failed
edge notes  d435i_ext:depth_failures=6  aurora_wrist:depth_failures=29
```

Wrist depth matching degrades over the run (29 failures) — watch the wrist
depth age/rate in the next run.

## Files

- `dual_verify5.json` — every observation/target/status/GPU-status message of
  the successful run.
- `mouse_observations_and_fused.json` — the 46 mouse observations and the 13
  fused samples.
- `cameras_dual_verify.yaml`, `fusion_dual_verify.yaml` — exact configs used
  (note the 1000 ms coalesce and the corrected external intrinsics).
- `edge_dual_verify5.log`, `gpu_dual_verify5.log`.

## Run 7 — coalesce experiment (the actual fix)

The 1000 ms test value was not a deployment answer, so the default was tested
against a hardware-appropriate profile:

| coalesce | duration | both views saw | two-view fused samples |
|---|---|---|---|
| **30 ms** (production default) | 42 s | cup 32 external / 26 wrist | **0** |
| **600 ms** (`config/fusion_dual_hardware.yaml`) | 42 s | cup 35 external / 31 wrist | **40** |

With 600 ms the edge produced two-view targets, e.g.:

```text
cup-034  t=33.83  pos (0.32974, -0.01871, -0.04227)
         weight_external 0.21 / weight_wrist 0.79
         depths 0.199 / 0.328 m   source_cameras [d435i_ext, aurora_wrist]
```

So the 30 ms default is not a sufficient pairing window for the asynchronous
2/4 fps uplink + GPU latency; 600 ms covers one external frame period plus the
per-camera inference skew. Cost: up to ~0.6 s of target-update latency, which
is acceptable for the current ~1.4 fps data path but not for a 10 Hz control
loop. The proper fix is capture-time pairing or a per-view bias-aware filter.

`config/fusion_dual_hardware.yaml` (new, in the repo) is `fusion.yaml` with
`coalesce_ms: 600.0` and this rationale. `config/fusion.yaml` keeps 30 ms so
the synthetic/low-latency path and the unit tests are unchanged; hardware
dual-camera runs must pass `fusion_config:=<path>/fusion_dual_hardware.yaml`.

## Still open after the coalesce fix: track drift

The 600 ms run still shows static tracks moving:

```text
cup-005   first (0.462, -0.042, -0.089) -> last (-0.298, 0.124, +0.144)   5 s
cup-034   first (0.326, -0.019, -0.039) -> last (0.430, -0.045, -0.077)   9 s
dining_table-006 first (0.320, -0.020, -0.069) -> (0.459, 0.269, -0.213)
```

The two views' bbox-centre points are different physical points (cup side vs
rim/top, plus the known external chain offset); the constant-velocity filter
with millimetre-scale measurement covariance interprets the alternating views
as velocity. Until this is fixed (per-view bias state, larger cross-view
measurement covariance, or a defined point/keypoint), fused **absolute**
positions must not be quoted, even though the fusion and weights now work.

Files: `dual_verify7.json`, `cup_observations_and_fused_600ms.json`,
`fusion_dual_hardware.yaml`, `edge_dual_verify7.log`, `gpu_dual_verify7.log`.

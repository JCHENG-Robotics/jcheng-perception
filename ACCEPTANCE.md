# Acceptance report — jcheng-perception

> Status: **inconclusive** as of 2026-11-26 for v0.1.0.
> Operator: JerichoCheng. Reviewer: pending.
> Scope: core/synthetic pipeline, Pi 5 build, wrist real data path, wrist TF,
> single-pose table-plane geometry, the cross-machine YOLO detection path,
> ADR 0004 aligned external depth, the split-edge ingress/fusion pipeline, one
> 60 s dual-view tag run, the D435i depth-recovery procedure and the offline
> ore-scene S0/S1 pipeline.
> Synthetic and partial hardware gates pass, including one dual-view run. The
> full dual-view window is not repeatable, multi-pose accuracy and motion
> integration have not run, so this is not a metric perception acceptance.
> No motion command was sent.

## 1. Gates

| Gate | Command | Level | Result | Evidence |
|---|---|---|---|---|
| Unit tests | `python3 -m pytest tests -q` | L1 | pass (93 tests) | `tests/` |
| Offline simulation | `python3 scripts/simulate_pipeline.py` | L3 | pass | `docs/EVIDENCE.md` |
| ROS build (host) | `colcon build --packages-select ...` | L0 | pass (4 packages) | CI `ros-build` |
| Synthetic end-to-end | `ros2 launch ... synthetic_demo.launch.py` | L3 | pass | 10 Hz, match 99%+ |
| Pi 5 ARM64 build | `colcon build` on Pi 5 | L0 | pass (4 packages, 27.1 s) | `docs/HARDWARE_BRINGUP.md` |
| Pi 5 synthetic end-to-end | `ros2 launch ... synthetic_demo.launch.py` | L3 | pass (6.2/6.3 Hz, 99.4%, 48 ms) | `docs/HARDWARE_BRINGUP.md` |
| Wrist real data path | Aurora driver + mock detector | L5 (data path only) | pass | `docs/HARDWARE_BRINGUP.md` |
| Wrist bus TF | `wrist_tf` at 10 Hz | L5 (TF only) | pass (1000+ reads, 0 failures) | `docs/HARDWARE_BRINGUP.md` |
| Table-plane check | static pose fit | L5 (single pose) | pass (1.1 mm, 1.21°, RMS 0.61 mm) | `docs/HARDWARE_BRINGUP.md` |
| Cross-machine YOLO | Pi 5 edge → laptop RTX 3060 | L5 (detection only) | pass (16 detections, stamp preserved) | `docs/EVIDENCE.md` |
| External aligned depth (ADR 0004) | single RealSense owner, aligned depth | L5 (topology) | pass — colour ~6.5 Hz / aligned depth ~3.9 Hz on the rebuilt image | `evidence/hardware_20261124_adr0004/`; `jcheng-system` deploy acceptance |
| Split edge | per-camera ingress + fusion, static run 4 | L5 (single static run) | pass — cross-view 1.55 mm; fusion max/p95/final 2.14 / 1.64 / 1.96 mm | `evidence/hardware_20261124_split_edge/` |
| Dual-view tag run 9 | two-view tag run, 60 s | L5 (single run) | pass — cross-view 3.37 mm; p95 3.78 mm; final 1.19 mm | `evidence/hardware_20261124_dualview/` |
| Dual-view repeatability run 10 | repeat 60 s dual-view | L5 | inconclusive — wrist depth reader wedged at 7.5 s | `CHANGELOG.md` |
| D435i depth recovery | restart `armpi-rs-depth`, reset-failed ingress | L5 (recovery) | pass — documented recovery | `docs/FAILURE_MODES.md` §4.1 |
| Ore scene S1 (offline) | `scripts/ore_scene.py` synthetic and public Mendeley baselines | L1–L3 | pass — synthetic/public baselines; real-ore accuracy Unverified | `evidence/ore_scene_s1_20261126/`; `docs/ORE_SCENE_S1.md` |
| Multi-pose / motion accuracy | — | L5 | not run | — |

## 2. Verified claims

| Claim | Level | Evidence |
|---|---|---|
| The edge preserves the capture timestamp and the GPU returns it unchanged | L1–L3 | `test_*`; synthetic demo |
| Timestamp matching succeeds at ≥99% in synthetic runs | L3 | 10 Hz synthetic; Pi 5 99.4% |
| One two-view fused track is produced in synthetic runs | L3 | `docs/EVIDENCE.md` |
| The Pi 5 ARM64 build and synthetic run work | L5 (build/synthetic) | 27.1 s, 6.2/6.3 Hz, 48 ms |
| The wrist Aurora RGB/depth stream is available | L5 (data path) | 6.7–6.8 Hz RGB, 3.1–6.4 Hz depth |
| The wrist frame comes from bus-measured joints, not the command cache | L5 | `wrist_tf` 10 Hz, 0 failures |
| The wrist camera table-plane geometry is consistent at one static pose | L5 (single pose) | z difference 1.1 mm, tilt 1.21°, RMS 0.61 mm |
| Real YOLO detections cross machines with the original stamp | L5 (detection) | 2.06 fps, 55 ms, 16 detections |
| The external camera now publishes colour plus aligned depth under one RealSense owner | L5 (topology) | ADR 0004; `docs/DEPLOYMENT.md`; deploy acceptance |
| The split-edge pipeline isolates per-camera DDS participants and survives an ingress restart | L5 (static run) | run 4; `evidence/hardware_20261124_split_edge/` |
| A single 60 s dual-view tag run produces metric observations from both views | L5 (single run) | run 9; `evidence/hardware_20261124_dualview/` |
| A missing D435i depth stream is recoverable without moving the arm | L5 (recovery) | `docs/FAILURE_MODES.md` §4.1 |
| No detector runs on the Pi 5 and no motion command is published | L0 | static tests; `docs/FAILURE_MODES.md` |

## 3. Not verified

| Item | Reason | Risk |
|---|---|---|
| Dual-view repeatability | Run 10 wrist depth reader wedged at 7.5 s | Full dual-view window is not repeatable |
| Multi-pose accuracy | Only one static arm pose was measured | Poses may not generalise; not cleared for motion |
| Real-ore accuracy / depth / multi-view / grasp | Offline ore-scene S1 uses synthetic and public baselines only | No real-ore product claim |
| Fixed-target and moving-target error | Not measured | No accuracy claim |
| 1 s occlusion recovery time | Not measured on hardware | Tracking recovery is unknown |
| GPU loss → COASTING → LOST timing | Not measured on hardware | Degradation timing is unknown |
| 30-minute stability, CPU/temperature/memory | Not run | Long-run stability is unknown |
| Domain isolation and telemetry mirroring | Not integrated with `jcheng-system` | Perception must not join the control domain |
| Motion integration | Not started | Poses are advisory only |

## 4. Limitations

- The synthetic and single-pose results are not a general accuracy claim.
- The external camera uses aligned depth (ADR 0004), but multi-pose metric
  accuracy has not been measured.
- The 2026-11-25 dual-view result is a single 60 s run; the wrist depth DDS
  reader can still wedge, so a repeatable full window is not established.
- The current poses must not be used for motion; integration must pass
  `jcheng_safety` and `jcheng_motion`.
- A container restart can leave the D435i depth unit closed; the recovered
  procedure restarts `armpi-rs-depth` and the ingress.
- Model weights and third-party runtimes carry their own licenses.

## 5. Sign-off

| Role | Name | Date |
|---|---|---|
| Operator | JerichoCheng | 2026-11-26 |
| Reviewer | pending | — |

## 6. Machine-readable report

None. Per-run reports, when produced, must validate against
`jcheng-interfaces/schemas/acceptance-report.v1.schema.json`.

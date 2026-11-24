# Acceptance report — jcheng-perception

> Status: **inconclusive** as of 2026-11-24 for v0.1.0.
> Operator: JerichoCheng. Reviewer: pending.
> Scope: core/synthetic pipeline, Pi 5 build, wrist real data path, wrist TF and
> single-pose table-plane geometry, and the cross-machine YOLO detection path.
> Synthetic and partial hardware gates pass; metric accuracy, aligned depth and
> motion integration have not run, so this is not a metric perception acceptance.
> No motion command was sent.

## 1. Gates

| Gate | Command | Level | Result | Evidence |
|---|---|---|---|---|
| Unit tests (v0.1.0) | `python3 -m pytest tests -q` | L1 | pass (58 tests) | `tests/` |
| Offline simulation | `python3 scripts/simulate_pipeline.py` | L3 | pass | `docs/EVIDENCE.md` |
| ROS build (host) | `colcon build --packages-select ...` | L0 | pass (4 packages) | CI `ros-build` |
| Synthetic end-to-end | `ros2 launch ... synthetic_demo.launch.py` | L3 | pass | 10 Hz, match 99%+ |
| Pi 5 ARM64 build | `colcon build` on Pi 5 | L0 | pass (4 packages, 27.1 s) | `docs/HARDWARE_BRINGUP.md` |
| Pi 5 synthetic end-to-end | `ros2 launch ... synthetic_demo.launch.py` | L3 | pass (6.2/6.3 Hz, 99.4%, 48 ms) | `docs/HARDWARE_BRINGUP.md` |
| Wrist real data path | Aurora driver + mock detector | L5 (data path only) | pass | `docs/HARDWARE_BRINGUP.md` |
| Wrist bus TF | `wrist_tf` at 10 Hz | L5 (TF only) | pass (1000+ reads, 0 failures) | `docs/HARDWARE_BRINGUP.md` |
| Table-plane check | static pose fit | L5 (single pose) | pass (1.1 mm, 1.21°, RMS 0.61 mm) | `docs/HARDWARE_BRINGUP.md` |
| Cross-machine YOLO | Pi 5 edge → laptop RTX 3060 | L5 (detection only) | pass (16 detections, stamp preserved) | `docs/EVIDENCE.md` |
| External aligned depth | ADR 0004 | L5 | not run | — |
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
| No detector runs on the Pi 5 and no motion command is published | L0 | static tests; `docs/FAILURE_MODES.md` |

## 3. Not verified

| Item | Reason | Risk |
|---|---|---|
| External aligned depth | The deployed topology is UVC colour + depth-only; ADR 0004 not executed | Metric fusion from the external camera is unavailable |
| Multi-pose accuracy | Only one static arm pose was measured | Poses may not generalise; not cleared for motion |
| Fixed-target and moving-target error | Not measured | No accuracy claim |
| 1 s occlusion recovery time | Not measured on hardware | Tracking recovery is unknown |
| GPU loss → COASTING → LOST timing | Not measured on hardware | Degradation timing is unknown |
| 30-minute stability, CPU/temperature/memory | Not run | Long-run stability is unknown |
| Dual-view repeatability | Run 10 wrist depth reader wedged at 7.5 s | Full dual-view window is not repeatable |
| Domain isolation and telemetry mirroring | Not integrated with `jcheng-system` | Perception must not join the control domain |
| Motion integration | Not started | Poses are advisory only |

## 4. Limitations

- The synthetic and single-pose results are not a general accuracy claim.
- The external camera is unaligned; unaligned depth yields no metric
  observation.
- The wrist depth DDS reader can wedge under load; a fresh participant may be
  needed.
- The current poses must not be used for motion; integration must pass
  `jcheng_safety` and `jcheng_motion`.
- Model weights and third-party runtimes carry their own licenses.

## 5. Sign-off

| Role | Name | Date |
|---|---|---|
| Operator | JerichoCheng | 2026-11-24 |
| Reviewer | pending | — |

## 6. Machine-readable report

None. Per-run reports, when produced, must validate against
`jcheng-interfaces/schemas/acceptance-report.v1.schema.json`.

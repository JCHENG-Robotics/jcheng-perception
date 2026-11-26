# jcheng-perception roadmap

> Status: **v0.1.0** as of 2026-11-24. Synthetic, Pi 5 and wrist-data-path
> evidence exists; metric accuracy, aligned depth and motion integration do not.

## Current state

| Item | State | Evidence |
|---|---|---|
| Core packages, synthetic pipeline and simulation | Complete (2026-11-23) | `tests/`; `scripts/simulate_pipeline.py` |
| Pi 5 ARM64 build and synthetic end-to-end | Complete (2026-11-23) | 6.2/6.3 Hz, match 99.4%, 48 ms |
| Wrist Aurora real data path | Complete (2026-11-23) | `docs/HARDWARE_BRINGUP.md` |
| Bus-measured wrist TF | Complete (2026-11-24) | 1000+ reads, 0 failures; `camera_optical_measured` |
| Single static-pose table-plane check | Complete (2026-11-24) | z difference 1.1 mm, tilt 1.21°, RMS 0.61 mm |
| Cross-machine real-YOLO detection path | Complete (2026-11-24) | 16 detections, capture stamp preserved, 0 depth matches |
| Aligned external depth (ADR 0004) | Not started | Split UVC colour + depth-only topology |
| Multi-pose and moving-target accuracy | Not started | — |
| Dual-view repeatability | In progress | Run 10 wrist depth reader wedged at 7.5 s; split-edge work ongoing |
| Motion integration through `jcheng-system` | Not started | — |
| Ore scene understanding S0 (research/architecture/interface) | Complete (2026-11-26) | `docs/ORE_SCENE_*.md`, `docs/SCENE_REPRESENTATION.md` |
| Ore scene understanding S1 (offline instance segmentation) | Complete, offline only (2026-11-26) | `evidence/ore_scene_s1_20261126/`; synthetic verified, real-ore accuracy Unverified |
| Ore S1.5 deep segmentation backend (SAM 2 / MobileSAM) | Planned | — |
| Real-ore accuracy / depth / multi-view / grasp | Not started | — |

## Milestones

| Milestone | Scope | State | Exit criterion | Evidence |
|---|---|---|---|---|
| M0 | Core, synthetic scene, offline simulation | Complete (2026-11-23) | Tests + simulation pass | `docs/EVIDENCE.md` |
| M1 | Pi 5 ARM64 + wrist real data path | Complete (2026-11-23) | 4 packages; wrist stream and mock detection | `docs/HARDWARE_BRINGUP.md` |
| M2 | Wrist TF and table-plane geometry | Complete, single pose (2026-11-24) | Bus TF at 10 Hz; table plane within 2 mm / 2° | `evidence/hardware_20261124_drift_verify/` |
| M3 | Cross-machine YOLO detection path | Complete (2026-11-24) | Capture stamp preserved; latency measured | `docs/EVIDENCE.md` |
| M4 | Aligned external depth and dual-view metric observations | Not started | Depth/colour aligned; non-zero match rate | ADR 0004 |
| M5 | Multi-pose and moving-target accuracy | Not started | Fixed-target error and tracking error within declared bounds | — |
| M6 | Supervised integration with the system/planner | Not started | Advisory pose consumed under `jcheng_safety`/`jcheng_motion`; no direct motion | — |

## Next

1. **M4 — align the external camera.** Execute ADR 0004: one RealSense owner,
   aligned depth, then re-run the dual-view tag test. Prerequisite: stop the
   second depth consumer during perception runs.
2. **Stabilize the wrist depth reader.** The DDS reader still wedges under load;
   the split per-camera ingress work is aimed at that. Until it is stable, a
   full dual-view 60 s window is not repeatable.
3. **M5 — multi-pose accuracy.** Repeat the table-plane and tag checks across
   several arm poses before any motion integration.

## Ore scene understanding track

This is a separate offline track; it does not change the M0–M6 runtime milestones
above and adds no DDS, device, depth or motion path.

| Stage | Scope | State | Exit criterion / evidence |
|---|---|---|---|
| S0 | Research, architecture, scene representation, interface design | Complete (2026-11-26) | Five Chinese design docs + S1 report; no contract change |
| S1 | Offline ore instance segmentation baseline | Complete, offline (2026-11-26) | 92 tests; selftest; synthetic AP50 0.368; public 40_70mm AP50 0.043 (domain-gap baseline) |
| S1.5 | SAM 2 / MobileSAM comparison on 6 GB | Planned | License/VRAM/AP parity; no weights in git |
| S2 | Visible 3D geometry (depth + TF) | Planned | Depth alignment + geometry repeatability; thresholds TBD by Board |
| S3 | Scene graph / scene state v2 | Planned | Evidence-backed relations, schema v2, density-safe runtime |
| S4 | Multi-view instance association | Planned | Association P/R/F1 with ambiguity, no forced IDs |
| S5 | Active perception / NBV | Planned | Simulation/offline first; advisory only |
| S6 | Grasp affordance | Planned | Offline/simulation; 5-DoF constraints; no motion |
| S7 | Applications / Learning / Benchmark integration | Planned | Contract proposal approved; offline end-to-end |
| S8 | Real ore sorting closed loop | Not started | Safety review + per-session authorization; `ExecuteMotion` only |

Start here: [`docs/ORE_SCENE_S1.md`](docs/ORE_SCENE_S1.md), then
[`docs/ORE_SCENE_ROADMAP.md`](docs/ORE_SCENE_ROADMAP.md).

## Out of scope

- Motion control, `cmd_vel`, joint or gripper commands.
- Safety decisions: workspace, limits and target freshness are the
  planner/safety layer's responsibility.
- Running YOLO on the Pi 5.
- Shipping model weights or vendor code.
- Opening camera devices: the vendor driver remains the single device owner.

## Dependencies

- `jcheng-interfaces` `>=0.1,<0.2`; the perception message package is
  provisional and will be promoted to the contract repository.
- `jcheng-system` for domain isolation, telemetry mirroring and the planner
  interface.
- Every hardware run requires separate, explicit operator authorization.

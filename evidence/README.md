# Evidence index

| Bundle | Date | Kind | Hardware claimed |
|---|---|---|---|
| `synthetic_20261123/` | 2026-11-23 | 53 unit tests (bundle date), offline fusion simulation, 4-package ROS build, one-machine synthetic end-to-end | **No** |
| `hardware_20261123/` | 2026-11-23 | Pi 5 ARM64 build + synthetic, wrist Aurora driver rebuild and real data-path test (mock blue detector; metric z invalid at the time) | **Yes, partial** |
| `hardware_20261124/` | 2026-11-24 | Measured wrist TF from bus-servo joints + live URDF FK; single-pose table-plane validation (z=-0.0836 m, RMS 0.61 mm) | **Yes, partial** |
| `hardware_20261124_yolo/` | 2026-11-24 | Cross-machine real YOLO (`yolov8s.pt`) on the external camera over CycloneDDS; detection path only, depth unaligned | **Yes, partial** |
| `hardware_20261124_plane_check/` | 2026-11-24 | Static table-plane check: wrist table-only fit -0.0828 m vs -0.0825 m reference; default ROI hit the curled tag sheet; external D435i plane 11 mm low | **Yes, partial** |
| `hardware_20261124_dualview/` | 2026-11-24 | Dual-camera real YOLO, first hardware two-view `FusedTarget` (`weight_external 0.67 / weight_wrist 0.33`, both sources); coalesce, fallback-intrinsics and GPU-reader findings | **Yes, partial** |
| `analysis_20261124_drift_bias/` | 2026-11-24 | Offline replay of the dual-view observations: cross-view bias model + same-view velocity; static drift 696 mm recorded / 478 mm old logic -> 6.4 mm | **No (re-analysis of recorded data)** |
| `hardware_20261124_drift_verify/` | 2026-11-24 | Deployed `eed1ca9` to the Pi and ran two bounded static dual-view captures (90 s / 60 s, no motion); worst required-label drift 4.7 mm / 8.1 mm vs 448-696 mm pre-fix; one 67 mm first-pair reference step documented | **Yes, static only** |
| `hardware_20261124_tag_verify/` | 2026-11-24 | Defined-point (AprilTag 36h11) static run: two-view tag centre 3.77 mm; fixed-point 5 mm gate 3.67 mm depth / 3.07 mm PnP after chain correction; no 67 mm first-pair step; velocity min-dt fix from the tag data | **Yes, static only** |
| `hardware_20261124_split_edge/` | 2026-11-24 | Per-camera ingress + fusion processes with 4 s watchdog/respawn; static 60 s: external 134 + wrist 226, cross-view 1.55 mm, fusion max 2.14 / p95 1.64 / final 1.96 mm; external ingress restarted at 25 s without losing the track | **Yes, static only** |
| `ore_scene_s1_20261126/` | 2026-11-26 | Offline ore S1: 93 tests, selftest (empty/bad/overlap), synthetic AP50 0.368 / AP50-95 0.211, public Mendeley 40_70mm held-out AP50 0.043 (domain-gap baseline); no DDS/device/depth/motion | **No** |
| `ore_scene_sam2_20261126/` | 2026-11-26 | Offline SAM 2.1 Tiny S1.5: 8-image AP50 0.508 (1008) / 0.637 (adaptive 504); 4-image independent split 0.6375; Rocks UAV AP50 0.301 (negative transfer); converter reproduces 0.6375; no DDS/device/depth/motion | **No** |
| `ore_scene_s2_20261126/` | 2026-11-26 | Offline S2 visible-geometry MVE on synthetic depth: 2500 points from a 1 m plane, visible centroid/AABB/OBB, zero-depth/empty-mask failures; 103 tests; no real depth/hardware | **No** |
| `ore_scene_s3_20261126/` | 2026-11-26 | Offline Scene State v2 prototype: v1->v2 geometry + touching/near relation edges, not_computed for missing geometry, forbidden true fields/reference/duplicate checks; no independent relation labels | **No** |
| `ore_scene_s3_relations_20261126/` | 2026-11-26 | Controlled 18-scene relation benchmark P/R/F1 1.0; e2e 39 synthetic scenes: classical match 43.5% / relation F1 0.067, SAM2.1 match 76.3% / F1 0.600; dense public 40_70mm 8 images (9,208 GT): classical match 10.4% / F1 0.030, SAM2.1 match 59.1% / F1 0.558; no real-ore relation accuracy | **No** |
| `ore_scene_s4_1_20261127/` | 2026-11-27 | Offline S4.1 dual-view generator: 4 scenes + 1 empty, RGB/z-depth/uint16 labels, 61 constructed cross-view pairs, 2 singletons; deterministic regeneration and mask/depth/pair consistency checks pass; no matcher/association metrics | **No** |
| `ore_scene_s4_2_20261127/` | 2026-11-27 | Offline S4.2 matcher on 8 dense synthetic scenes / 125 pairs: mutual projection P/R/F1 1.000/0.944/0.971 vs centroid baseline 0.973/0.872/0.920; 0 vs 10 ambiguous; forced-ID 0; synthetic only | **No** |
| `ore_scene_s4_3_20261127/` | 2026-11-27 | Offline S4.3 visible-geometry fusion: 118/118 associated clusters fused, 0 ambiguous, 23 singletons; info-weighted visible centroids, AABB/OBB, per-view members; `jcheng.scene_association.v1` validation + JSONL round-trip pass; synthetic only | **No** |
| `ore_scene_s4_4_20261127/` | 2026-11-27 | Offline S4.4 pre-registered sensitivity: mixed synthetic 8+1 scenes / 122 pairs, baseline P/R/F1 1.000/0.934/0.966; tz 10 mm extrinsic error drops F1 to 0.827/0.876 (10 mm acceptance failed); 100 ms offset fails closed; ablations exploratory; synthetic only | **No** |
| `ore_scene_s4_4b_20261127/` | 2026-11-27 | Offline S4.4b calibration-aware depth gate, pre-registered config frozen before an independent held-out synthetic seed `20261128`: baseline P/R/F1 1.000/0.952/0.976 (126 pairs), all 10 mm and six 1-degree checks pass, 100 ms offset fails closed; post-hoc uninformed tz 10 mm 0.875/0.880 vs 0.946/0.950 informed; synthetic only, `1σ=10 mm` is an assumption | **No** |

The synthetic bundle proves the data path and logic, not robot accuracy. See
[`docs/EVIDENCE.md`](../docs/EVIDENCE.md) for the full narrative and the list of
unverified items.

Add a new dated directory for every hardware session. Each bundle must contain:

- `summary.json` — machine-readable results and environment;
- logs/commands actually used;
- explicit `hardware_claimed` flag;
- calibration/config hashes when hardware is involved.

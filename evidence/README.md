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

The synthetic bundle proves the data path and logic, not robot accuracy. See
[`docs/EVIDENCE.md`](../docs/EVIDENCE.md) for the full narrative and the list of
unverified items.

Add a new dated directory for every hardware session. Each bundle must contain:

- `summary.json` — machine-readable results and environment;
- logs/commands actually used;
- explicit `hardware_claimed` flag;
- calibration/config hashes when hardware is involved.

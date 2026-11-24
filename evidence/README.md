# Evidence index

| Bundle | Date | Kind | Hardware claimed |
|---|---|---|---|
| `synthetic_20261123/` | 2026-11-23 | 53 unit tests (bundle date), offline fusion simulation, 4-package ROS build, one-machine synthetic end-to-end | **No** |
| `hardware_20261123/` | 2026-11-23 | Pi 5 ARM64 build + synthetic, wrist Aurora driver rebuild and real data-path test (mock blue detector; metric z invalid at the time) | **Yes, partial** |
| `hardware_20261124/` | 2026-11-24 | Measured wrist TF from bus-servo joints + live URDF FK; single-pose table-plane validation (z=-0.0836 m, RMS 0.61 mm) | **Yes, partial** |
| `hardware_20261124_yolo/` | 2026-11-24 | Cross-machine real YOLO (`yolov8s.pt`) on the external camera over CycloneDDS; detection path only, depth unaligned | **Yes, partial** |

The synthetic bundle proves the data path and logic, not robot accuracy. See
[`docs/EVIDENCE.md`](../docs/EVIDENCE.md) for the full narrative and the list of
unverified items.

Add a new dated directory for every hardware session. Each bundle must contain:

- `summary.json` — machine-readable results and environment;
- logs/commands actually used;
- explicit `hardware_claimed` flag;
- calibration/config hashes when hardware is involved.

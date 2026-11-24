# 2026-11-24 — table-plane check (static, no motion)

Robot: ArmPi Ultra, Pi 5 (ROS 2 Humble, container `ArmPiUltra`).
Condition: container up, `armpi-vision` + `armpi-rs-depth` active, `wrist_tf`
and `ext_tf` publishing, wrist depth `/depth_cam/depth/image_raw` ~1.9 Hz.
The arm was **stationary**; no motion command was sent. One static pose only.

## Result

| Fit | table z in `base_link` | tilt | RMS | points |
|---|---|---|---|---|
| reference (`docs/CALIBRATION.md`, 2026-11-20) | -0.0825 m | — | — | — |
| script default (**central 40 % ROI**) | **-0.0751 m** | 0.74° | 0.75 mm | 2535 |
| table only (`--roi-frac 0.95 --exclude 145,505,80,360`) | **-0.0828 m** | 1.00° | 0.7 mm | 6.5–6.7 k |

Camera origin `(0.1805, 0.0091, 0.1923)` m, view z `(+0.498, -0.003, -0.867)`.
The clean table-only fit agrees with the reference to **0.3 mm** → the wrist
TF chain (`base_link -> camera_optical_measured`) passes at this pose.

The default-ROI number is **not** a TF error: the fixed central 40 % ROI landed
on the printed AprilTag sheet, which is not lying flat. Offline refit of the
same depth frame (`wrist_snap_depth.png`, depth `CameraInfo`):

```text
central 40 % (script ROI)      -0.0750 m
whole tag sheet (10795 pts)    -0.0803 m   (rms 1.1 mm)
table without sheet (11708)    -0.0825 m   (rms 0.6 mm)
table strips: left -0.0836 / right -0.0819 / top -0.0829
```

The sheet's centre sits ~7.5 mm above the table and its edges ~2–4 mm above:
the sheet is curled / not flat. `wrist_snap_rgb.png` shows the printed tag
sheet filling the central ROI.

## External chain (D435i) cross-check

One aligned-depth frame (`ext_aligned_depth.png`, 640×480, same stamp in depth
and colour), `base_link -> camera_color_optical_frame` from `ext_tf` +
driver, `K = (606.50, 606.60, 325.03, 249.19)`:

```text
left 45 %   table_z = -0.0940 m  tilt 0.23°  rms 0.7 mm
centre      table_z = -0.0936 m  tilt 0.37°  rms 0.6 mm  (13455 pts)
right 55 %  table_z = -0.0916 m  tilt 0.71°  rms 1.2 mm
```

The external chain puts the same table **~11 mm lower** than the wrist chain
(-0.0936 vs -0.0825) with a similar tilt. This matches the cross-view z error
measured on the target in run 3 (wrist − external = +9.4…+10.6 mm) and does not
look like a pitch error (the fitted tilt would change); it is a pose/z offset in
the external chain. The wrist is also the chain that agrees with the reference
at this pose, so the **external chain stays the prime suspect** for the run-3
cross-view FAIL.

Caveat: the reference -0.0825 itself originates from the 2026-11-20 external
calibration. Either the external camera physically moved since then or its
deployed extrinsic/depth path is offset; separating those needs a fresh
external calibration (chessboard on the same table), not another plane fit.
Likely contributor: the deployed `frames.yaml` entry is still the UVC-camera
`T_base_front`, composed with the driver's D435i colour optical offset by
`ext_tf`; the D435i colour lens was mounted in ADR 0004 and was never
calibrated against the table itself, so an ~11 mm lens offset would show up
exactly like this.

## Script fix made during this check

`scripts/wrist_tf_multipose_check.py`:

- `--info-topic` default changed from `/depth_cam/rgb/camera_info` to
  `/depth_cam/depth/camera_info`. The depth image is 640×400 with
  `fx=423.02`; the RGB feed has `fx=418.05`. Using the RGB intrinsics on the
  same frame reads **-0.0789 m instead of -0.0750 m** (3.9 mm low). The
  historical multipose z values (-0.0859…-0.0868 m) were produced with the old
  default; if they were run without an explicit `--info-topic` they are ~4 mm
  too negative.
- added `--roi-frac` and `--exclude u0,u1,v0,v1` so the fit can avoid raised
  sheets/parts. The 0.40 default is unchanged.

## Caveat that matters for the fixed-point runs

The tag sheet is curled by up to ~7 mm. The tag's measured z therefore depends
on where on the sheet it is and how the sheet is laid down — enough to move the
cross-view z comparison by several mm. Flatten or replace the sheet before
using the tag for a cross-view (<5 mm) decision.

## Files

- `wrist_plane_roi40_20261124.csv` — 21 valid samples, default ROI.
- `wrist_plane_tableonly_20261124.csv` — 26 valid samples, table-only fit.
- `wrist_snap_rgb.png`, `wrist_snap_depth.png` — the frame analysed above.
- `ext_aligned_depth.png` — external aligned depth preview.
- `summary.json`, `commands.md`, `hashes.txt`.

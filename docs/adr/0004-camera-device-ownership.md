# ADR 0004 — D435i device ownership and depth/color alignment

- Status: proposed (requires a supervised hardware decision)
- Date: 2026-11-23

## Context

The chassis D435i is currently consumed by two processes on the robot:

1. `armpi-ext-cam.service` → `ext_cam_node` owns `/dev/video4` as a UVC camera,
   publishes `/ext_cam/color/image_raw`, `/ext_cam/color/camera_info` and the
   measured `base_link -> ext_cam_optical` static TF. The Studio gateway follows
   this ROS topic (the legacy direct-UVC mode is not deployed).
2. `armpi-rs-depth.service` → `realsense2_camera` with `enable_color:=false`
   publishes `/camera/camera/depth/image_rect_raw` (16UC1,
   `camera_depth_optical_frame`, 640×480, 4–8 Hz with colour running).

This split exists because the UVC colour path was needed for the Studio preview
before the ROS topic existed, and because opening colour first makes librealsense
fail with `EBUSY`. The consequence for perception is serious: the depth image is
**not registered to the colour image**, it has its own optical frame and its own
intrinsics, and the two streams are not hardware-synchronised. A detection box in
colour cannot be back-projected with the depth pixel at the same coordinate.

The repository's default `config/cameras.yaml` assumes a single RealSense node
publishing colour plus `aligned_depth_to_color`, which is **not** the currently
deployed topology.

## Decision (proposed)

Adopt one owner for the D435i: `realsense2_camera` with colour and aligned depth.

- `realsense2_camera` owns the USB device.
- Publish `/camera/color/image_raw`, `/camera/color/camera_info`,
  `/camera/aligned_depth_to_color/image_raw`,
  `/camera/aligned_depth_to_color/camera_info`, `camera_color_optical_frame`.
- Set `align_depth.enable:=true`; the aligned depth uses the colour intrinsics
  and `camera_color_optical_frame`.
- Point the Studio gateway's `ARMPI_GATEWAY_AUX_CAMERA_TOPIC` at
  `/camera/color/image_raw`; retire `armpi-ext-cam.service` (the gateway's
  `RosImageCamera` already supports this).
- Keep `armpi-rs-depth.service` as the single launch unit, renamed/updated to
  enable colour as well.

Perception then uses the default `cameras.yaml` profile unchanged: one optical
frame, one intrinsic matrix, metric depth registered to the detection pixels.

## Why not keep the split and register in software

A software registration path is possible (reproject the colour ray into the
depth frame with `T_depth_color`, iterate the depth estimate), but it adds:

- a second optical frame and a second intrinsic matrix per camera;
- dependence on the colour/depth extrinsics and on their time offset;
- a second TF chain and more failure modes;
- an accuracy penalty from unsynchronised streams.

For a coarse external view this might be acceptable, but it duplicates logic that
`align_depth_to_color` already provides in the driver, and it makes the wrist and
external cameras behave differently.

## Consequences

Positive:

- Pixel-to-depth correspondence is exact by construction.
- One device owner; no `EBUSY` ordering dance for colour.
- The repository's default config and tests match the deployed topology.
- The Studio preview keeps working through the ROS topic.

Negative / risks:

- With colour + depth + alignment, the depth rate measured on this robot is
  about 4–8 Hz (vs ~10.5 Hz depth-only). The external view is a coarse view, so
  this is acceptable, but it must be measured after the change.
- The D435i USB3 bandwidth and the container's librealsense patch must be
  re-verified.
- The measured `T_base_front` was solved with the UVC colour stream. The
  RealSense colour sensor is the same physical camera, but the extrinsic must be
  re-validated (fixed target, compare external vs wrist) before metric use.
- `armpi-ext-cam.service` is retired; the Studio gateway configuration and
  deployment docs must change together.

## Hardware verification required

1. `fuser -v /dev/video4` shows exactly one owner.
2. `/camera/color/image_raw` and `/camera/aligned_depth_to_color/image_raw`
   have the same stamp family and the same `camera_color_optical_frame`.
3. Fixed target: external and wrist observations agree at the calibration error
   level; table z is consistent.
4. Measure depth rate/FPS with colour + alignment enabled; update
   `max_uplink_fps` and matching tolerances if below 10 Hz.
5. Studio `/aux-camera/status` reports the ROS source, not `uvc:/dev/video4`.

Until this ADR is accepted and executed, the split topology must **not** be used
for metric fusion. A temporary split profile would need the software registration
work described above.

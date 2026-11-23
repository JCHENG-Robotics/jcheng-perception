# License review — pending

This repository does **not** ship a `LICENSE` file. Copyright is reserved;
JCHENG-Robotics license review is in progress, consistent with the other
organization repositories.

## What is in this repository

- Original Python and ROS 2 source written for this project.
- Configuration values and measured extrinsics from `jcheng-manipulation`
  (internal project evidence).
- No model weights, no vendor binaries, no third-party source copies.

## Third-party runtime dependencies

| Dependency | Use | License note |
|---|---|---|
| ROS 2 / rclpy / tf2 / cv_bridge | runtime | Apache-2.0 |
| OpenCV | image codec, mock detector | Apache-2.0 |
| NumPy | numerics | BSD-3-Clause |
| PyYAML | configuration | MIT |
| Ultralytics | optional GPU detector backend | **AGPL-3.0 by default**; commercial licensing may be required |
| OpenVINO / ONNX Runtime | optional model backends | vendor terms |

## Model weights

Weights are deliberately not committed. The `gpu.yaml` paths point outside the
repository. Anyone adding a weight file must record its origin, license and
export format in this file before committing it. Ultralytics `YOLO` model
weights are subject to the Ultralytics license, not this repository's pending
license.

## Vendor and baseline material

The `jcheng-manipulation` repository is a frozen reference baseline. This
repository reuses its published frame names and calibration numbers only; it
does not copy vendor code.

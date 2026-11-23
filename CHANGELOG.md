# Changelog

## 0.1.0 — 2026-11-23

Initial distributed-perception implementation.

- Four ROS 2 packages: `jcheng_perception_msgs`, `jcheng_perception_core`,
  `jcheng_perception_edge`, `jcheng_perception_gpu`.
- Pi 5 edge: JPEG uplink with capture timestamps, per-camera depth and TF
  timestamp rings, capture-time matching, depth back-projection, association,
  distance-weighted two-view fusion, constant-velocity Kalman tracking.
- Laptop GPU: asynchronous two-stream YOLO with bounded per-camera queues,
  hot reload, keypoint support and capture-timestamp preservation.
- Measured timings on an x86 Jazzy host: edge callbacks < 1 ms, 10 Hz per
  camera, 99% time-match success, one fused track with both views.
- 43 ROS-free unit tests; four-package colcon build; synthetic end-to-end demo.
- No hardware deployment and no real YOLO model in this repository.

## Version policy

`major` changes a topic/message contract incompatibly; `minor` adds a
capability or camera; `patch` fixes a defect or documentation. The message
package is provisional until promoted to `jcheng-interfaces`.

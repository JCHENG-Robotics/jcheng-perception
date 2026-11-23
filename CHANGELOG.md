# Changelog

## Unreleased

- Shadow detector launch/config: a second GPU node reads the same uplink and
  publishes to `/perception/*_shadow`, with its own status topic. Production
  fusion is unaffected.
- Detector backends: Ultralytics, ONNX Runtime (CPU), OpenVINO IR (CPU), OpenCV
  DNN (legacy models) and mock. ONNX Runtime, OpenVINO and Ultralytics CPU were
  installed in an isolated venv and verified on the vendor models:
  `yolov8s.onnx` 197.5 ms, `yolov8s.xml` 126.7 ms, `yolov8s.pt` 1.04 s first
  frame (4 detections each); OBB `yolov8n-obb.pt` also passed.
- `python_executable` launch argument and `scripts/setup_gpu_venv.sh` for
  running the ROS node with a venv runtime; ONNX Runtime and Ultralytics were
  verified in-node (232.8 ms and 154.3 ms steady per frame on CPU).
- CUDA PyTorch installed and verified: `torch 2.9.1+cu128`,
  `torchvision 0.24.1+cu128`, RTX 3060; `yolov8s.pt` 9.0 ms FP32 / 7.4 ms FP16 steady per image,
  ROS node 54.3 ms/frame average. A torch 2.11/NCCL mismatch was resolved by
  pinning the 2.9.1 pair; `RUNTIME=ultralytics-cuda` reproduces it.
- `scripts/check_gpu_backend.py`: ROS-free detector smoke check for every
  backend; reports missing runtime/model explicitly.
- Detector adapter tests (mock, unknown backend, missing model) — 53 tests total.
- ADR 0004: proposed single RealSense owner with aligned depth; documents why
  the currently deployed split UVC-colour + depth-only topology cannot be used
  for metric fusion.

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
- 53 ROS-free unit tests; four-package colcon build; synthetic end-to-end demo.
- No hardware deployment and no real YOLO model in this repository.

## Version policy

`major` changes a topic/message contract incompatibly; `minor` adds a
capability or camera; `patch` fixes a defect or documentation. The message
package is provisional until promoted to `jcheng-interfaces`.

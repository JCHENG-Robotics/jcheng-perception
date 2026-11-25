# GPU perception image — real-robot smoke (2026-11-25)

> Status: **hardware smoke PASS for the ArUco and Ultralytics backends.** The
> production wrist pose model is not on this laptop, so the YOLO run used a
> stock COCO `yolov8n.pt` and makes no accuracy claim. No motion was commanded;
> the node and container were stopped after the captures.

## Environment

| Part | Value |
|---|---|
| Laptop | Ubuntu 24.04, driver 595.99.02, `nvidia-container-toolkit` 1.20.2 + CDI |
| Image | `jcheng-perception-gpu:latest`, `sha256:25a185ec95bb…`, 14.5 GB |
| GPU | RTX 3060 Laptop, visible in the container through `--gpus all` |
| Robot | Pi 5, wired `10.42.0.218`; split edge stack active (2 ingress + fusion + TF) |
| ROS | Humble on both sides, CycloneDDS on the wired link, domain 0 |

## What ran

1. **Tag**: `config/gpu_tag.yaml` (60 mm AprilTag 36h11, id 0), ~8 s.
2. **YOLO**: temporary `/tmp/gpu_yolo_smoke.yaml` with a stock
   `yolov8n.pt` (80 COCO classes) on both cameras, `device: "0"`, ~8 s.

The node was launched with:

```bash
docker run -d --name jcheng-perception-gpu --network host --gpus all \
  -v "$HOME/jcheng/humble_ws:/workspace" \
  -v "$HOME/jcheng/jcheng-perception:/repo:ro" \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///workspace/cyclonedds-wired.xml \
  -e ROS2CLI_NO_DAEMON=1 -e HOME=/tmp \
  jcheng-perception-gpu:latest sleep infinity

docker exec jcheng-perception-gpu bash -lc \
  'source /opt/ros/humble/setup.bash && source /workspace/tag_ws/install/setup.bash && \
   ros2 launch jcheng_perception_gpu laptop_gpu.launch.py \
     gpu_config:=<config> python_executable:=/opt/jcheng/venv/bin/python'
```

## Results

| Backend | Camera FPS (ext/wrist) | Inference ms (ext/wrist) | Frames (ext/wrist) | Dropped | Edge `gpu_alive` | Tracks |
|---|---|---|---|---|---|---|
| `aruco` (60 mm tag) | 5.41 / 5.36 | **2.94 / 2.69** | 44 / 43 | 0 / 0 | `true` | 1 active, 1 confirmed |
| `ultralytics` (`yolov8n.pt`) | 6.15 / 5.74 | **8.46 / 8.80** steady (31 ms first-window average includes model load) | 95 / 96 | 16 / 22 | `true` | 1 active, 1 confirmed |

The edge status during the tag run: `total_frames=1708`, all matched,
`match_success_rate=1.0`, `mean_detection_latency_ms=892.8`,
`d435i_ext:obs_age_ms=160`, `aurora_wrist:obs_age_ms=135`.

GPU evidence from inside the container:

```text
487, /opt/jcheng/venv/bin/python, 210 MiB
```

The same process was the running `perception_gpu` node, so the Ultralytics
steady-state 8.5 ms is GPU inference, not a CPU fallback.

## Camera driver degradation (same session)

The wrist observation rate was low in both runs (0.58–1.5 fps). The ingress
log explained it: over 22 s the Aurora stream delivered **175 RGB frames but
only 58 depth frames** (~7.95 / 2.64 Hz), and the edge's depth match then
rejected most detections. The vendor driver had been up for 12 h.

After `sudo systemctl restart armpi-vision` (the container itself is just
`tail -f /dev/null`, so no arm motion), the same probe measured:

| Stream | Before (12 h uptime) | After restart |
|---|---|---|
| Aurora RGB | 7.95 Hz | **10.92 Hz** |
| Aurora depth | 2.64 Hz | **10.83 Hz** |

`scripts/check_camera_rates.py` then reported 11.00 / 10.83 Hz
(`camera_rates_after_restart.txt`). The ingress watchdog respawns its process
after the driver returns; `docs/DEPLOYMENT.md` records the 6/5 Hz preflight
thresholds and the restart procedure.

## Raw files

| File | Content |
|---|---|
| `gpu_status_tag.yaml` | `GpuStatus` from the tag run |
| `edge_status_tag.yaml` | edge `PerceptionStatus` during the tag run |
| `gpu_status_yolo.yaml` | `GpuStatus` from the stock-YOLO run |
| `edge_status_yolo.yaml` | edge `PerceptionStatus` during the YOLO run |
| `nvidia_smi_yolo.txt` | compute process and GPU memory |
| `camera_rates_after_restart.txt` | Aurora RGB/depth rates after the driver restart |
| `gpu_node_tag.log`, `gpu_node_yolo.log` | node startup logs |
| `image_id.txt` | image digest and byte size |

## Non-claims

- The YOLO run used stock `yolov8n.pt`, **not** `/opt/jcheng/models/ext_yolov8n.pt`
  or `wrist_grasp_pose.pt`; it validates the image, CUDA, the Ultralytics
  backend and the DDS path, not detection quality.
- The wrist ingress rate was low during these runs (0.58–1.5 fps). The tag run
  still had fresh wrist observations (135 ms); the YOLO run's wrist
  `obs_age_ms` reached 74 s because the stock model did not detect a tracked
  object in that view.
- No arm, base or gripper command was sent. No production node was left
  running; the container was removed after the captures.
- A stale host GPU node from 2026-11-24 21:16 (Jazzy + host venv,
  `gpu_dual.yaml`) was still publishing to `/perception/detections/*` with
  repeated DDS write errors. It was stopped before this run, and the GPU topics
  reported `Publisher count: 0` afterwards.

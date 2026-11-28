# Perception GPU node in Docker (laptop / Orin)

> Status: **Jazzy Dockerfile prepared, not rebuilt or GPU-verified.** The host
> has driver 595.99.02 but no `nvidia-container-toolkit`, so
> `docker run --gpus all` is unavailable and installing the toolkit needs the
> user's sudo password. The local `jcheng-perception-gpu:latest` image is still
> the old Humble build; rebuild it from this Dockerfile before use.

## Why

The production GPU config (`config/gpu.yaml`) uses `backend: ultralytics`.
The GPU node runs on the laptop/Orin with ROS 2 Jazzy, matching the host
toolchain, so this image is Jazzy and no longer needs a separate host venv.
The Pi 5 edge remains ROS 2 Humble inside the vendor container, so the
edge↔GPU link is cross-distro again; see `docs/FAILURE_MODES.md` for the
CycloneDDS caveat.

## Host prerequisite (once, needs sudo)

Docker on this laptop currently lists only `runc`:

```bash
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Verify before building anything:

```bash
docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu24.04 nvidia-smi
```

Driver 595.99.02 supports the CUDA 12.8 wheels this image installs.

## Build

From the repository root:

```bash
docker build -f deploy/Dockerfile.gpu -t jcheng-perception-gpu:latest deploy
```

The first build downloads several GB (torch + CUDA wheels); the verified host
venv is 6.8 GB. The image does **not** install `opencv-python` — the apt
`python3-opencv` build carries ArUco, and both the tag and YOLO backends
must use it.

## Build the Jazzy workspace

The same workspace layout as the tag run:

```bash
docker run -d --name jcheng-perception-gpu --network host --gpus all \
  -v "$HOME/jcheng/humble_ws:/workspace" \
  -v "$HOME/jcheng/jcheng-perception:/repo:ro" \
  -v /opt/jcheng/models:/opt/jcheng/models:ro \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///workspace/cyclonedds-wired.xml \
  -e ROS2CLI_NO_DAEMON=1 -e HOME=/tmp \
  jcheng-perception-gpu:latest sleep infinity

# symlink the three packages into /workspace/tag_ws/src, then:
docker exec jcheng-perception-gpu bash -lc \
  'source /opt/ros/jazzy/setup.bash && cd /workspace/tag_ws && \
   colcon build --symlink-install \
     --packages-select jcheng_perception_msgs jcheng_perception_core jcheng_perception_gpu'
```

## Run the node

```bash
docker exec jcheng-perception-gpu bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /workspace/tag_ws/install/setup.bash && \
   ros2 launch jcheng_perception_gpu laptop_gpu.launch.py \
     gpu_config:=/repo/ros2_ws/src/jcheng_perception_gpu/config/gpu.yaml \
     python_executable:=/opt/jcheng/venv/bin/python'
```

Models stay outside the repository at `/opt/jcheng/models/`, as `gpu.yaml`
already expects, and are mounted read-only.

## Verify

```bash
# GPU visible in the container
docker exec jcheng-perception-gpu bash -lc \
  '/opt/jcheng/venv/bin/python -c "import torch; print(torch.__version__, torch.cuda.is_available())"'

# cv2 is the apt build with ArUco, not a pip wheel
docker exec jcheng-perception-gpu bash -lc \
  '/opt/jcheng/venv/bin/python -c "import cv2; print(cv2.__version__, cv2.__file__, hasattr(cv2, \"aruco\"))"'

# no-ROS backend smoke test
docker exec jcheng-perception-gpu bash -lc \
  '/opt/jcheng/venv/bin/python /repo/scripts/check_gpu_backend.py \
     --backend ultralytics --model /opt/jcheng/models/ext_yolov8n.pt \
     --image /path/to/sample.jpg'
```

## Limits

- Nothing here has been run on hardware yet; the Jazzy image has not been
  rebuilt or GPU-verified.
- The image inherits the plain `ros:jazzy-ros-base` tools; it does not include
  MoveIt, so it cannot replace `armpi-humble-moveit` for the arm.
- The Pi 5 edge is still Humble. Cross-distro CycloneDDS (`invalid data size`
  warnings, reader wedge) is a known risk; prefer keeping both ends on the same
  distro where possible.
- `--network host` is required for the wired CycloneDDS configuration; the
  container must not be started while another GPU node is publishing the same
  detection topics.

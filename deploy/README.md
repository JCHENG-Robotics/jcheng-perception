# Perception GPU node in Docker (laptop)

> Status: **image prepared, not built or GPU-verified.** The host has driver
> 595.99.02 but no `nvidia-container-toolkit`, so `docker run --gpus all` is
> unavailable and installing the toolkit needs the user's sudo password. The
> tag/ArUco path continues to run in `armpi-humble-moveit`; this image is for
> the `ultralytics` (YOLO) backend.

## Why

The production GPU config (`config/gpu.yaml`) uses `backend: ultralytics`.
Until now that node ran from the host `~/.venvs/jcheng-perception-gpu` under
Jazzy, so the laptop and the Pi edge were on different ROS 2 distributions.
The 2026-11-24 same-distro runs moved the ArUco/tag path into Humble and
removed that cross-distro link; this image does the same for YOLO.

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
docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu22.04 nvidia-smi
```

Driver 595.99.02 supports the CUDA 12.8 wheels this image installs.

## Build

From the repository root:

```bash
docker build -f deploy/Dockerfile.gpu -t jcheng-perception-gpu:latest deploy
```

The first build downloads several GB (torch + CUDA wheels); the verified host
venv is 6.8 GB. The image does **not** install `opencv-python` — the apt
`python3-opencv` 4.5.4 build carries ArUco, and both the tag and YOLO backends
must use it.

## Build the Humble workspace

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
  'source /opt/ros/humble/setup.bash && cd /workspace/tag_ws && \
   colcon build --symlink-install \
     --packages-select jcheng_perception_msgs jcheng_perception_core jcheng_perception_gpu'
```

## Run the node

```bash
docker exec jcheng-perception-gpu bash -lc \
  'source /opt/ros/humble/setup.bash && source /workspace/tag_ws/install/setup.bash && \
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

- Nothing here has been run on hardware yet; the `armpi-humble-moveit` tag path
  remains the verified Humble route.
- The image inherits the plain `ros:humble-ros-base` tools; it does not include
  MoveIt, so it cannot replace `armpi-humble-moveit` for the arm.
- `--network host` is required for the wired CycloneDDS configuration; the
  container must not be started while another GPU node is publishing the same
  detection topics.

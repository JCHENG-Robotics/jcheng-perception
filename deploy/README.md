# Perception GPU node in Docker (laptop)

> Status: **built, GPU-verified and hardware-smoke-tested on 2026-11-25.**
> Image `jcheng-perception-gpu:latest` (14.5 GB) runs with `--gpus all`:
> torch 2.9.1+cu128 reports CUDA 12.8 with `cuda_available()`, `cv2` is the
> apt 4.5.4 build with ArUco, and `jcheng_perception_{msgs,core,gpu}` build in
> `/workspace/tag_ws`. Against the live robot the tag backend ran at
> 2.94/2.69 ms and a stock `yolov8n.pt` at 8.46/8.80 ms on the RTX 3060, with
> the edge reporting `gpu_alive=true` for both; see
> `evidence/hardware_20261125_gpu_image/`. The production wrist pose model is
> still absent, so the YOLO run is an infrastructure check, not an accuracy
> one. The tag/ArUco path also still runs in `armpi-humble-moveit`.

## Why

The production GPU config (`config/gpu.yaml`) uses `backend: ultralytics`.
Until now that node ran from the host `~/.venvs/jcheng-perception-gpu` under
Jazzy, so the laptop and the Pi edge were on different ROS 2 distributions.
The 2026-11-24 same-distro runs moved the ArUco/tag path into Humble and
removed that cross-distro link; this image does the same for YOLO.

## Host prerequisite (once, needs sudo)

The Ubuntu archive has no `nvidia-container-toolkit`; add NVIDIA's repository
first. Docker 29 resolves `--gpus` through CDI, so the CDI spec is required —
without it the error is `failed to discover GPU vendor from CDI: no known GPU
vendor found`.

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
sudo nvidia-ctk cdi generate --output=/var/run/cdi/nvidia.yaml
```

Installed on this laptop on 2026-11-25 (toolkit 1.20.2). Verify:

```bash
docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu22.04 nvidia-smi
```

Driver 595.99.02 supports the CUDA 12.8 wheels this image installs.

## Build

From the repository root:

```bash
docker build -f deploy/Dockerfile.gpu -t jcheng-perception-gpu:latest deploy
```

The built image is 14.5 GB. The first build downloads several GB of CUDA
wheels; `PIP_INDEX_URL` (default: the Tsinghua PyPI mirror) can be overridden
with `--build-arg`. The image does **not** install `opencv-python` — the apt
`python3-opencv` 4.5.4 build carries ArUco, and both the tag and YOLO backends
must use it. `torch` is the PyPI 2.9.1 Linux wheel, which bundles CUDA 12.8.

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

## Pre-run checks

```bash
# weights: the repository does not ship them; this lists what is missing
docker exec jcheng-perception-gpu bash -lc \
  '/opt/jcheng/venv/bin/python /repo/scripts/check_models.py \
     --config /repo/ros2_ws/src/jcheng_perception_gpu/config/gpu.yaml --load'

# stock-COCO fallback for infrastructure tests only
docker exec jcheng-perception-gpu bash -lc \
  '/opt/jcheng/venv/bin/python /repo/scripts/check_models.py \
     --config /repo/ros2_ws/src/jcheng_perception_gpu/config/gpu_fallback.yaml --load'

# Aurora stream rates (BEST_EFFORT); want RGB >= 6 Hz and depth >= 5 Hz
docker exec jcheng-perception-gpu bash -lc \
  'source /opt/ros/humble/setup.bash && \
   python3 /repo/scripts/check_camera_rates.py --duration 10'
```

The Aurora driver degrades after a long uptime: on 2026-11-25 the depth stream
fell to 2.64 Hz after 12 h and returned to 10.83 Hz after
`sudo systemctl restart armpi-vision` (~2.5 min). The ingress watchdog respawns
its process when the driver returns.

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

- The hardware smoke used a stock `yolov8n.pt`, not the production
  `ext_yolov8n.pt` / `wrist_grasp_pose.pt`. Both runs were perception-only;
  nothing on the arm, base or gripper was commanded.
- The image inherits the plain `ros:humble-ros-base` tools; it does not include
  MoveIt, so it cannot replace `armpi-humble-moveit` for the arm.
- `--network host` is required for the wired CycloneDDS configuration; the
  container must not be started while another GPU node is publishing the same
  detection topics.

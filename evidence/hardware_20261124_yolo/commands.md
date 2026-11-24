# Commands and raw results — 2026-11-24 cross-machine real YOLO

No motion commands were sent. The robot and laptop were on CycloneDDS domain 0
over the wired link (10.42.0.1 <-> 10.42.0.218).

## 1. Laptop RMW

```bash
sudo apt-get install -y ros-jazzy-rmw-cyclonedds-cpp
# ros-jazzy-cyclonedds 0.10.5, ros-jazzy-rmw-cyclonedds-cpp 2.2.4
```

Wired-only CycloneDDS config (laptop interface is the USB Ethernet adapter):

```xml
<CycloneDDS>
  <Domain Id="0">
    <General>
      <Interfaces>
        <NetworkInterface name="enx00e04ca373c7" priority="1" multicast="true"/>
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
```

Verified:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclonedds-laptop.xml
export ROS2CLI_NO_DAEMON=1
ros2 topic list          # robot topics visible
ros2 topic echo /joint_states --once --field name
# ['joint1', ..., 'r_out_joint']
```

## 2. External camera

```bash
ros2 topic hz /ext_cam/color/image_raw
# average rate 13.2-14.4 Hz
```

The external camera publishes `bgr8` 1280x720. A captured frame was used to
check the model offline on the RTX 3060:

```text
yolov8s.pt: chair 0.337, dining table 0.215
```

## 3. Edge start (bounded)

```bash
# inside the robot container
source /opt/ros/humble/setup.bash
source /home/ubuntu/jcheng-perception/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
timeout -s INT 60 ros2 launch jcheng_perception_edge pi5_edge.launch.py \
  cameras_config:=/home/ubuntu/jcheng_hw/cameras_ext_hw.yaml \
  start_wrist_tf:=false
```

`cameras_ext_hw.yaml` uses `jpeg_quality: 55`, `max_uplink_fps: 3.0`, and a
dummy wrist entry. The intentional high-resolution load is bounded to 60 s.

## 4. GPU start

```bash
source /opt/ros/jazzy/setup.bash
source /home/jerichocheng/jcheng/jcheng-perception/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclonedds-laptop.xml
timeout -s INT 40 ros2 launch jcheng_perception_gpu laptop_gpu.launch.py \
  gpu_config:=/tmp/gpu_ext_real.yaml \
  python_executable:=$HOME/.venvs/jcheng-perception-gpu/bin/python
```

Log:

```text
[INFO] [jcheng_perception_gpu]: GPU detector ready: d435i_ext
[INFO] [jcheng_perception_gpu]: detector loaded: yolov8s.pt:1775123732000000000
```

## 5. First detection

`ros2 topic echo /perception/detections/d435i_ext --once --qos-reliability best_effort`:

```text
header:
  stamp: {sec: 1795487500, nanosec: 703345576}
  frame_id: ext_cam_optical
camera_id: d435i_ext
model_id: yolov8s.pt:1775123732000000000
frame_seq: 1795487500703345576
image_width: 1280
image_height: 720
inference_ms: 9.456453323364258
detections:
- label: dining table
  class_id: 60
  score: 0.22965675592422485
  bbox_xyxy: [3.42578125, 45.28558349609375, 1276.867919921875, 707.2860107421875]
```

The detection stamp equals `frame_seq` and is the original capture time.

## 6. Edge status

```text
camera_ids: [d435i_ext, aurora_wrist]
camera_fps: [4.387145519256592, 0.0]
total_frames: 115
total_detections: 16
detections_matched: 0
detections_dropped: 16
match_success_rate: 0.0
mean_detection_latency_ms: 186.79690551757812
gpu_alive: true
notes: [d435i_ext:depth_failures=25]
```

Zero matches are expected: the colour stream is 1280x720 UVC, the depth stream
is the unaligned 640x480 RealSense stream. This bundle validates the detection
path only; metric fusion needs ADR 0004.

## 7. GPU status

```text
camera_ids: [d435i_ext]
fps: [2.0593512058258057]
inference_ms: [55.201446533203125]
frames_received: [37]
frames_dropped: [4]
frames_failed: [0]
model_ids: [yolov8s.pt:1775123732000000000]
```

## 8. Incident and cleanup

The first unbounded attempt coincided with the Pi becoming unresponsive at the
application layer while ICMP still worked (SSH banner and gateway timed out).
That command also contained a pkill pattern that matched its own shell, so
whether the edge started is unproven. The robot was power-cycled; the bounded
retry above completed and left the vendor services active.

After the bounded test:

```text
edge processes: 0
gpu processes: 0
load average: 5.83 (4 cores)
systemctl is-active start_node armpi-ext-cam armpi-rs-depth armpi-gateway
  -> active x4
```

A repeated SIGINT during teardown used to print a `KeyboardInterrupt`
traceback in the edge/GPU/synthetic nodes. The nodes now ignore SIGINT while
joining workers; `timeout -s INT` on the synthetic demo exits with zero
tracebacks and all three processes report "finished cleanly".

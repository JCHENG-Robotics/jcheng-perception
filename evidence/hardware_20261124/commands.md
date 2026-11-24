# Commands and raw results — 2026-11-24 wrist TF

All commands ran inside the `ArmPiUltra` container on the Pi 5. No motion
commands were sent. `ROS_DOMAIN_ID=0`, CycloneDDS on both ends.

## 1. Source and build

```bash
cd /home/ubuntu/jcheng-perception/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jcheng_perception_edge
# Summary: 1 package finished [18.0 s]
```

The source overlay was commit `523cb8d` (the container's git checkout was not
changed; the tree was updated from a tar archive).

## 2. Start `wrist_tf`

```bash
source ~/.zshrc
source /home/ubuntu/jcheng-perception/ros2_ws/install/setup.zsh
ros2 run jcheng_perception_edge wrist_tf --ros-args \
  -p config_file:=/home/ubuntu/jcheng-perception/ros2_ws/install/jcheng_perception_edge/share/jcheng_perception_edge/config/wrist_tf.yaml
```

Log excerpt:

```text
[INFO] [wrist_tf]: wrist_tf: base_link -> camera_optical_measured at 10 Hz, service /ros_robot_controller/bus_servo/get_state
[INFO] [wrist_tf]: wrist_tf chain base_link -> link4: joint1 -> joint2 -> joint3 -> joint4
[INFO] [wrist_tf]: wrist_tf reads=1 failures=0 q=[joint1=-0.017 joint2=+0.406 joint3=-1.382 joint4=-1.629] camera=(+0.175, +0.008, +0.197) view_z=(+0.512, -0.007, -0.859)
[INFO] [wrist_tf]: wrist_tf reads=1038 failures=0 q=[joint1=-0.017 joint2=+0.411 joint3=-1.386 joint4=-1.629] camera=(+0.175, +0.008, +0.197) view_z=(+0.512, -0.007, -0.859)
```

Bus read latency (5 servos, 10 calls):

```text
median 34.3 ms, max 39.1 ms
```

## 3. TF check

```bash
ros2 run tf2_ros tf2_echo base_link camera_optical_measured
```

```text
At time 1795482629.323541866
- Translation: [0.175, 0.008, 0.197]
- Rotation: in RPY (degree) [-149.414, 0.142, -91.051]
- Matrix:
 -0.018 -0.861  0.509  0.175
 -1.000  0.017 -0.007  0.008
 -0.002 -0.509 -0.861  0.197
  0.000  0.000  0.000  1.000
```

## 4. Table-plane check

The diagnostic script is stored beside this file as
[`check_wrist_plane.py`](check_wrist_plane.py). It reads one depth frame and the
depth `CameraInfo`, transforms sampled pixels with
`base_link -> camera_optical_measured`, and fits a plane with RANSAC.

```bash
python3 /tmp/check_wrist_plane.py
```

```text
exact-stamp lookup failed: Lookup would require extrapolation into the future.
  Requested time 1795482735.829000 but the latest data is at time 1795482735.823544
tf_frame source: latest
camera_position_m: [0.1753, 0.0082, 0.1973]
camera_view_z: [0.5124, -0.0073, -0.8587]
camera_height_m: 0.1973
depth_stamp: 1795482735 829000000
depth_frame: depth_camera_link
depth_shape: (400, 640) K: [[423.02, 0.0, 320.1], [0.0, 423.83, 199.9], [0.0, 0.0, 1.0]]
valid_samples: 4676 base_z_range: -0.082 -0.069
plane_normal: [-0.0162, -0.0136, 0.9998]
plane_z_at_base_origin_m: -0.0836
plane_tilt_deg: 1.21
inlier_fraction: 1.0
plane_rms_mm: 0.61
camera_above_plane_m: 0.2809
```

The exact-stamp lookup fell back to the latest TF because the depth frame was
5.5 ms newer than the newest TF sample. The arm was stationary; this does not
affect the plane result.

## 5. Edge/GPU probe

`pi5_edge.launch.py` and `laptop_gpu.launch.py` (mock detector) ran while the
camera was open. The blue target was no longer in view, so the probe exercised
the stream/TF path but produced no detections:

```text
camera_ids: [d435i_ext, aurora_wrist]
camera_fps: [0.0, 4.911839962005615]
total_frames: 16
total_detections: 0
gpu_alive: true
notes: [aurora_wrist:depth_failures=3]
```

The data path with detections was already exercised on 2026-11-23; the TF fix
is the subject of this bundle.

## 6. Cleanup

```bash
bash /home/ubuntu/jcheng_hw/stop_hw.sh
bash /home/ubuntu/jcheng_hw/stop_vision.sh
pkill -f 'wrist_tf --ros-args'
pkill -f 'ros2 run jcheng_perception_edge wrist_tf'
pkill -f 'ros2 launch jcheng_perception'
```

Checked afterwards:

```text
pgrep -af 'vision.launch|aurora930|perception_edge|perception_gpu|wrist_tf'
  -> no live process (aurora930_node remained defunct until reaped)
ros2 topic info /depth_cam/rgb/image_raw
  -> Unknown topic (no publisher)
systemctl is-active start_node armpi-ext-cam armpi-rs-depth armpi-gateway
  -> active x4
```

Vendor services and systemd units were not modified.

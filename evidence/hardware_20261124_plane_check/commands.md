# Commands used — 2026-11-24 table-plane check

Everything ran from the laptop to the Pi 5 and inside the `ArmPiUltra`
container. `ROS_DOMAIN_ID=0`, CycloneDDS with
`CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml`. No motion commands.

## 1. State probe

```bash
ssh pi@10.42.0.218 'docker ps --format "{{.Names}}\t{{.Status}}"
  systemctl is-active armpi-vision armpi-rs-depth armpi-ext-cam
  pgrep -af "aurora930_node|realsense2_camera|perception_edge|wrist_tf|ext_tf"'

docker exec -u ubuntu ArmPiUltra /bin/bash -lc "
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
  timeout 8 ros2 topic hz /depth_cam/depth/image_raw
  timeout 6 ros2 topic echo --once /depth_cam/depth/camera_info --field k
  timeout 6 ros2 topic echo --once /depth_cam/rgb/camera_info --field k
  timeout 10 ros2 run tf2_ros tf2_echo base_link camera_optical_measured"
```

Measured: `/depth_cam/depth/image_raw` ~1.9 Hz; depth K
`fx=423.02 fy=423.83 cx=320.10 cy=199.90`; RGB K
`fx=418.05 fy=419.10 cx=308.78 cy=187.13`;
`base_link -> camera_optical_measured` = `(0.180, 0.009, 0.192)` m.

## 2. Default-ROI check

```bash
# script copied host -> Pi -> container with ssh cat + docker cp
ssh pi@10.42.0.218 'cat > /tmp/wrist_plane_check.py' < scripts/wrist_tf_multipose_check.py
ssh pi@10.42.0.218 'docker cp /tmp/wrist_plane_check.py ArmPiUltra:/tmp/'

docker exec -u ubuntu ArmPiUltra /bin/bash -lc "
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
  timeout -s INT 45 python3 /tmp/wrist_plane_check.py \
    --duration 30 --rate 1 \
    --info-topic /depth_cam/depth/camera_info \
    --out /tmp/wrist_plane_20261124.csv"
```

Result: `table_z=-0.0751 m`, tilt 0.74°, rms 0.8 mm, 2534 points.

## 3. Snapshot of the ROI content

```bash
# /tmp/snap_wrist.py: one RGB + one depth frame, prints encoding/stamp, writes npy+ppm
docker exec -u ubuntu ArmPiUltra /bin/bash -lc "
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
  timeout 25 python3 /tmp/snap_wrist.py /depth_cam/rgb/image_raw \
    /depth_cam/depth/image_raw /tmp/wrist_snap"

# rgb bgr8 640x400 stamp 1795511315.513, depth mono16 640x400 same stamp
```

Offline refit (numpy, laptop) on `/tmp/wrist_snap_depth.npy` with the depth
intrinsics and the TF row from the CSV: central 40 % `-0.0750`, whole tag sheet
`-0.0803`, table without the sheet (`u 145..505, v 80..360` excluded)
`-0.0825`. Table strips: left `-0.0836`, right `-0.0819`, top `-0.0829`.
With the RGB intrinsics on the same depth frame the central fit reads `-0.0789`.

## 4. Table-only check

```bash
docker exec -u ubuntu ArmPiUltra /bin/bash -lc "
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
  timeout -s INT 45 python3 /tmp/wrist_plane_check.py \
    --duration 30 --rate 1 --roi-frac 0.95 --exclude 145,505,80,360 \
    --out /tmp/wrist_plane_tableonly_20261124.csv"
```

Result: `table_z=-0.0828 … -0.0830 m`, tilt 1.0°, rms 0.7 mm, 6.5–6.7 k points.

## 5. External D435i cross-check

```bash
# /tmp/snap_ext_plane.py: one aligned-depth frame + CameraInfo + TF lookup
docker exec -u ubuntu ArmPiUltra /bin/bash -lc "
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
  timeout 30 python3 /tmp/snap_ext_plane.py \
    /camera/aligned_depth_to_color/image_raw \
    /camera/aligned_depth_to_color/camera_info \
    base_link camera_color_optical_frame /tmp/ext_plane"

# tf ok t=(0.1393, 0.0270, 0.0367)
# K = (606.50, 606.60, 325.03, 249.19), 640x480, 89.5 % valid
```

Plane fits in `base_link`: left 45 % `-0.0940`, centre `-0.0936`
(13455 pts, rms 0.6 mm), right 55 % `-0.0916`.

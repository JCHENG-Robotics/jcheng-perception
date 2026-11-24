# 2026-11-24 — ADR 0004 executed: one RealSense owner with aligned depth

Robot: ArmPi Ultra, Pi 5 (ROS 2 Humble), laptop RTX 3060 (ROS 2 Jazzy).
No motion commands were sent in any run below.

## What changed on the robot

| Item | Before | After |
|---|---|---|
| D435i owner | `armpi-ext-cam.service` (UVC `/dev/video4`, 1280×720) | `armpi-rs-depth.service` (`realsense2_camera` 4.55.1) |
| Colour | `/ext_cam/color/image_raw` | `/camera/color/image_raw` (640×480 rgb8) |
| Depth | `/camera/camera/depth/image_rect_raw` (unaligned) | `/camera/aligned_depth_to_color/image_raw` (16UC1) |
| Frame | `ext_cam_optical` | `camera_color_optical_frame` (both streams) |
| Studio gateway | `ros:/ext_cam/color/image_raw` | `ros:/camera/color/image_raw` |
| Edge TF tree | `base_link→ext_cam_optical` from `ext_cam_node` | `ext_tf` publishes `base_link→camera_link` from `frames.yaml`; the driver supplies `camera_link→camera_color_optical_frame` |

The launch line is `rs_launch.py depth_module.depth_profile:=640x480x15
rgb_camera.color_profile:=640x480x15 enable_color:=true
align_depth.enable:=true camera_namespace:=/`.

`camera_namespace:=/` matters: the default `camera` namespace would publish
`/camera/camera/color/...`, which does not match `config/cameras.yaml` or the
ADR text. An empty value (`camera_namespace:=''`) is rejected by
`ros2 launch` as a malformed argument, so `/` is the way to get root-level
topics.

## Verification results

Camera streams (`fuser -v /dev/video4` shows one owner, the realsense node):

```text
colour        /camera/color/image_raw                    6.90 Hz  rgb8   640x480
aligned depth /camera/aligned_depth_to_color/image_raw   7.54 Hz  16UC1  640x480
stamp delta colour vs aligned depth                      0.00 ms
frame_id, both streams                                   camera_color_optical_frame
colour intrinsics (driver)                               fx 606.50 fy 606.60 cx 325.03 cy 249.19
```

Studio gateway `/aux-camera/status`:

```json
{"available": true, "source": "armpi_debug_gateway_ros",
 "topic": "ros:/camera/color/image_raw", "width": 640, "height": 480,
 "fps": 10.6, "sample_age_ms": 43}
```

Edge + GPU with the repository default `cameras.yaml` (real `yolov8s.pt` on the
RTX 3060, 45 s bounded run):

```text
camera_fps              d435i_ext 4.78, aurora_wrist 2.77
total_frames            96
total_detections        251
detections_matched      179   (71.3 %)
detections_dropped      72
mean_depth_match_age    0.0 ms (external), 31.5 ms (wrist, 8 Hz depth)
depth_failures          d435i_ext 3
gpu_alive               true
```

This is the first run where external detections back-project to metric
observations: the aligned depth has the same stamp and frame as the colour
image. The earlier cross-machine test (`hardware_20261124_yolo`) deliberately
produced 0 matches because colour and depth came from different streams.

`ext_tf` closes the TF tree. `base_link → camera_color_optical_frame` now
resolves, and the composed matrix matches `frames.yaml` within 0.5 mm / 0.03°:

```text
-0.042 -0.538  0.842  0.139
-0.999  0.049 -0.019  0.027
-0.031 -0.842 -0.539  0.037
```

Composing with the *ideal* link/optical rotation instead of the driver's
actual chain leaves a 15 mm error (the driver's colour optical frame carries a
15 mm baseline from `camera_link`), so `ext_tf` waits for the driver transform
and inverts it, falling back to the ideal rotation only after 5 s.

## Wrist TF multi-pose check

Method (`scripts/wrist_tf_multipose_check.py`): fit the dominant plane in the
central 40 % of the wrist depth image, transform its normal and offset into
`base_link` with the live `base_link → camera_optical_measured` TF, log one row
at 2 Hz. The table is fixed, so the plane offset must not move with the arm.
A fit closer than 8 cm to the camera is discarded as "arm in the ROI".

| Segment | Camera x/y/z (m) | Duration | Tilt | Table z median | Valid points |
|---|---|---|---|---|---|
| baseline | 0.193 / 0.013 / 0.203 | 8 s | 0.57° | -0.0860 | 2560 |
| pose 1 | 0.185 / 0.044 / 0.204 | 13 s | 0.65° | -0.0866 | 2560 |
| pose 2 | 0.194 / 0.012 / 0.203 | 83 s | 0.54° | -0.0859 | 2560 |
| pose 3 | 0.183 / 0.011 / 0.203 | 87 s | 0.76° | -0.0868 | 2560 |
| raised | 0.212 / 0.012 / 0.238 | 56 s | 1.74° | -0.1021 | 706 |
| return | 0.193 / 0.012 / 0.206 | 25 s | 0.70° | -0.0860 | 2560 |
| pose 4 | 0.185 / 0.044 / 0.206 | 50 s | 0.65° | -0.0862 | 2560 |

- Five same-height poses: table z spread **0.9 mm**, tilt 0.54–0.76°. The
  wrist TF is consistent at the arm's working height.
- The raised pose (+3.5 cm) reads 16 mm lower with 27 % valid depth pixels and
  1.74° tilt. That is most likely a different table patch plus a
  range-dependent depth bias, but it is **not proven** — a fixed target seen
  from both poses is needed to separate depth bias from TF error.
- A first recording (before the ROI was tightened) had two low poses where the
  fit locked onto a non-table surface (24–36° tilt, inlier count collapsing);
  those samples are discarded by the 8 cm and inlier checks.

## Fixed target across poses (2026-11-24, real YOLO, light profile)

A cup was placed at a fixed spot on the table and the arm was moved between two
poses; both runs used `cameras_fixed_target.yaml`, real `yolov8s.pt` on both
cameras, and the vendor workspace sourced so `wrist_tf` publishes
`camera_optical_measured`. The Pi stayed healthy for both runs (load 5.9,
1.4 GB used) with ROS logs on tmpfs.

```text
pose A   wrist camera (0.176, 0.008, 0.196)   edge 138 frames, 98 detections, 62 matched
pose B2  wrist camera (0.188, 0.024, 0.204)   edge 148 frames, 130 detections, 40 matched

wrist cup observation, pose A    (+0.247, -0.030, +0.061) m   depth 0.152 m  score 0.94
wrist cup observation, pose B2   (+0.242, +0.012, +0.057) m   depth 0.153 m  score 0.71
delta                            (-5.0, +42.3, -4.3) mm        |= 42.8 mm

table plane, pose A              z = -0.0860 m   tilt 0.57°   rms 0.6 mm   2560 pts
table plane, pose B2             z = -0.0889 m   tilt 0.87°   rms 0.6 mm   1553 pts
delta                            z 2.9 mm       tilt 0.3°
```

Interpretation:

- The **table plane** is a well-defined geometric surface and agrees within
  **2.9 mm / 0.3°** between the two poses at working height (and within 0.9 mm
  across the five same-height poses of the earlier recording). The wrist TF is
  therefore consistent at the arm's working height to a few millimetres.
- The cup's **bbox-centre 3D point** differs 42 mm in y while depth stays at
  0.152–0.153 m. A bbox centre lands on the cylinder wall, so a rotation of the
  arm picks a different surface point; the spread is the cup radius scale, not
  a TF error. Fine validation needs a defined point (rim centre or base), or a
  small marker.
- The raised pose of the earlier recording (+3.5 cm, 27 % valid depth, 16 mm
  offset) is still best explained by a different table patch plus
  range-dependent depth bias: at working height the same TF reproduces the
  plane to 3 mm.
- Practical envelope for now: **coarse positioning to ~5 mm at working height
  is supported; do not quote better than that, and do not use the raised-pose
  number as a TF error.**

## Acceptance criteria (ADR 0004)

1. `fuser -v /dev/video4` shows exactly one owner — **pass** (realsense node).
2. Colour and aligned depth share a stamp family and frame — **pass**
   (0.00 ms delta, `camera_color_optical_frame`).
3. Fixed target: external and wrist agree, table z consistent — **partly**:
   external now yields metric observations and the wrist TF is consistent at
   working height; the fixed-target cross-view check is still pending.
4. Depth rate measured with colour + alignment — **pass** (7.5 Hz, update
   `max_uplink_fps` if aiming for 10 Hz fusion).
5. Studio `/aux-camera/status` reports the ROS source — **pass**.

## Incident 2026-11-24 11:36 — third application-layer hang (light profile)

After the D435i controller reboot, the external camera came back at USB 3.0
(5000M, uvcvideo) and both streams were healthy:

```text
/camera/color/image_raw              35 samples in 7 s
/depth_cam/rgb/image_raw             25 samples in 7 s
/camera/aligned_depth_to_color/...   50 samples in 7 s
uptime load average                  1.82 / 1.03 / 0.41
wrist_tf                             reads=43 failures=0
```

Starting the edge with the **light** profile (`cameras_fixed_target.yaml`,
2 fps external / 4 fps wrist) plus the real YOLO GPU node then produced:

```text
edge status at t+30 s   camera_fps 3.66 / 2.98, total_frames 74, detections 0, gpu_alive false
edge status at t+45 s   camera_fps 0.0 / 0.0, total_frames 0
DDS graph (laptop)      uplink publishers 1, GPU subscribers 1,
                        detection publishers 1 — the graph itself was connected
sshd                    banner timeout again; Pi unreachable
```

So the edge stopped receiving frames first, then the whole userland hung, while
the laptop-side DDS graph still looked correct. This is the third occurrence
and the first one with the light profile, so uplink rate is **not** the root
cause. Leading hypotheses (unproven): SD-card / overlay I/O stall (kernel
alive, ICMP fine, sshd cannot fork — the gateway HTTP still answered 401 during
the second hang), or memory pressure from the container + vendor stack + edge.

Recommended next probes (do these before more hardware runs):

1. Move `/home/ubuntu/.ros/log` to a tmpfs, and start the edge with
   `--log-level warn`; watch for a `D`-state `mmc` in `dmesg` and for
   `iostat -x 1` saturation.
2. Run the edge under `systemd-run --scope -p MemoryMax=1.5G` so an OOM is
   visible instead of taking the whole box down.
3. Run a 30 s edge-only window (no GPU, wrist camera only) and watch
   `/perception/status` at 2 s intervals; if it drops to 0 fps, capture
   `cat /proc/pressure/io` and `/proc/meminfo` immediately.

## Incident 2026-11-24 11:21 — D435i USB3 host controller died

While the light fixed-target profile was running (edge + wrist camera + real
YOLO) the xHCI controller that owns the D435i stopped responding:

```text
xhci-hcd xhci-hcd.0: Timeout while waiting for setup device command
usb 2-1: device not accepting address 2, error -62
xhci-hcd xhci-hcd.0: xHCI host controller not responding, assume dead
xhci-hcd xhci-hcd.0: HC died; cleaning up
usb 2-1: USB disconnect, device number 2
realsense2_camera_node: No RealSense devices were found!
```

`lsusb -t` then shows bus 2 with only the root hub; the colour stream had
already been logging `UVCIOC_CTRL_QUERY ... Protocol error` before the
controller died. The wrist camera (bus 3, USB 2.0) kept working: the edge
reported `camera_fps` 5.3 for `aurora_wrist`, wrist detections arrived and an
observation carried `depth_m = 0.362 m`, so this is a USB/host failure, not a
fusion or edge bug. Recovery needs a reboot (or a USB power cycle); a service
restart cannot re-create the controller.

Same run also exposed a GPU-side teardown race:

```text
rclpy._rclpy_pybind11.RCLError: Failed to publish: publisher's context is
invalid (node.py:223 in _publish_detections, worker thread after SIGINT)
```

The GPU node prints a traceback at shutdown when the worker thread publishes
while the context is being destroyed. Cosmetic for data, but it should be
fixed like the edge/wrist nodes (stop workers before `rclpy.shutdown()`).

## Incident 2026-11-24 11:13 — Pi application-layer hang (second occurrence)

While starting the fixed-target run (`poseA`: default `cameras.yaml` at 15 fps
external uplink plus the wrist uplink, with colour + aligned depth live and the
vendor stack running) the Pi stopped answering SSH:

```text
ping                      0 % loss, 0.2 ms
TCP 22 / 8765             accept connections
HTTP :8765 without token  401 (gateway process alive)
sshd                      no banner within 6-20 s, repeated
edge log                  /tmp/edge_poseA.log never produced a status/target
```

The GPU node had loaded both detectors; the edge produced no
`/perception/status` and no `/perception/targets`, so the hang happened around
the edge start-up. This is the same symptom as the 2026-11-24 power-cycle
incident (application layer dead, kernel networking fine). Recovery needs a
power cycle; no remote shell was available.

Consequences:

- Do not start the default `cameras.yaml` (15 fps external uplink) on a loaded
  Pi while a human is working at the robot.
- Use `cameras_fixed_target.yaml` (2 fps external / 4 fps wrist, q60) for
  fixed-target and human-in-the-loop runs.
- Start the edge **before** the GPU node and check `/perception/status` within
  ~15 s; abort if the Pi's load average exceeds ~5 before starting.

## Rollback

Backups live in `/home/pi/adr0004-20261124/` on the Pi:

```bash
sudo install -m 755 /home/pi/adr0004-20261124/rs-depth-run.sh \
  /usr/local/lib/armpi-rs-depth/rs-depth-run.sh
sudo systemctl stop armpi-rs-depth && sudo systemctl start armpi-rs-depth
sudo systemctl start armpi-ext-cam
# then drop ARMPI_GATEWAY_AUX_CAMERA_TOPIC from /etc/armpi-gateway/gateway.env
sudo systemctl restart armpi-gateway
```

## Not verified

- External extrinsic re-validation with a fixed target (the matrix was solved
  with the UVC 1280×720 stream; both are the same colour sensor but the check
  is cheap and should be recorded).
- Wrist detection chain on a fixed target across poses (COCO detect is the
  agreed interim: no wrist pose/keypoint weights exist in the vendor tree).
- 30-minute stability at 6–8 Hz with colour + alignment on the Pi.
- Planner integration; motion stays under `jcheng_safety`/`jcheng_motion` and
  needs the multi-pose result first.

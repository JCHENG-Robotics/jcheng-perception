# 实机接入记录（只读/相机，2026-11-23 → 2026-11-24）

> 本次没有发送任何运动命令，没有修改厂商容器或 systemd 服务；只启动了相机/感知进程并在结束后停止。
> 结论：**Pi 5 ARM64 构建与合成链路通过；腕部相机数据链路通过；腕部度量 TF 的根因已定位并修正，
> 单静态姿态桌面平面核验通过（z 差 1.1 mm）。** 外机位深度仍未对齐，运动集成仍未启用。

## 1. 机器人当前拓扑

| 项 | 状态 |
|---|---|
| Pi 5 / 容器 | Raspberry Pi 5 8 GB，`ArmPiUltra`（ROS 2 Humble，CycloneDDS，域 0） |
| 厂商栈 | `start_node`、`armpi-ext-cam`、`armpi-rs-depth`、`armpi-gateway` 均 active |
| 外机位 | `ext_cam_node` UVC 彩色（`/ext_cam/color/image_raw`，7.5–9.8 Hz） + 仅深度 RealSense（`/camera/camera/depth/image_rect_raw`，3.7–4.9 Hz） |
| 腕部相部 | Aurora 930：RGB 6.7–6.8 Hz，深度 3.1–6.4 Hz，640×400 |
| TF | `base_link` 动态树存在；`/tf` reliable，`/tf_static` 有多个 transient-local publisher |

外机位的彩色与深度来自两个进程，**没有对齐**；不能直接做同一像素反投影。见
[ADR 0004](adr/0004-camera-device-ownership.md)。

## 2. Pi 5 ARM64 构建与合成端到端

四个包在容器内构建通过（27.1 s），合成场景（edge + mock GPU）结果：

```text
camera_fps: 6.207 / 6.343
total_frames: 174, detections: 170, matched: 169, dropped: 0
match_success_rate: 0.9941
mean_detection_latency_ms: 48.09
phase: FINE, active_tracks: 1, confirmed_tracks: 1
weights: external 0.2203 / wrist 0.7797
edge timing: image 1.4–1.7 ms, depth 0.3–0.4 ms, detections 0.9–1.0 ms,
             publish 1.6–3.6 ms, tf 0.4–0.7 ms
```

这证明分布式逻辑在 Pi 5 ARM64 上可运行；合成场景的渲染/GPU mock 与 edge 同机，帧率低于
x86 主机的 10 Hz。

## 3. 腕部相机恢复

容器内 `deptrum-ros-driver-aurora930` 的 install 是空壳（缺 ament 索引/可执行文件），
但源码和本地 aarch64 SDK 在：

```text
/home/ubuntu/third_party_ros2/third_party_ws/src/deptrum-ros-driver-aurora930-0.2.1001
```

在**独立 install/build 目录**重建（未覆盖 vendor 安装）：

```bash
cd /home/ubuntu/third_party_ros2/third_party_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select deptrum-ros-driver-aurora930 \
  --cmake-args -DSTREAM_SDK_TYPE=AURORA930 \
  --build-base /home/ubuntu/jcheng_aurora_build \
  --install-base /home/ubuntu/jcheng_aurora_install
```

随后 `vision.launch.py` 需要同时 source `ros2_ws` 和独立 install。相机恢复：RGB
6.7–6.8 Hz、深度 3.1–6.4 Hz，内参与 2026-11-17 标定一致；`rgb_camera_link` TF
可查到。测试结束后已停止相机与感知进程，`/depth_cam/rgb/image_raw` publisher 回到 0。

## 4. 真实腕部数据链路（部分成功）

用蓝色方块 + mock 蓝色阈值检测器（不是 YOLO），edge/GPU 均在 Pi 5 本地、
CycloneDDS 域 0：

```text
GPU: fps 4.89, frames_received 39, failed 0, inference 3.22 ms
Edge: camera_fps 8.32, detections 47, matched 26, dropped 21 (depth_failures 20)
      match_success_rate 0.553, latency 71.46 ms, depth_age 34.5 ms
Target: blue_block-001, phase FINE, wrist weight 1.0, one confirmed track
Position: (0.1071, -0.0602, 0.5911) m
```

数据路径、压缩上行、采集时间戳回溯、深度/TF 匹配、关联与融合都已实际执行；GPU
0 失败。当次目标 z=+0.59 m 明显高于桌面；根因已在 2026-11-24 定位为命令缓存 TF
（见下节），数据路径本身有效。

## 5. 腕部度量 TF：根因与修正（2026-11-24）

### 5.1 根因：`/joint_states` 是命令缓存

同一静止姿态下同时读取：

| 源 | joint1 | joint2 | joint3 | joint4 | joint5 |
|---|---|---|---|---|---|
| `/joint_states`（`joint_state_publisher` / 控制器回显） | +0.8378 | 0 | 0 | 0 | 0 |
| 总线伺服 `get_state`（脉冲换算） | -0.013…-0.017 | +0.406…+0.411 | -1.382 | -1.629 | ≈0 |

用命令缓存组合 `T_base_link←camera_optical_calibrated`，相机 z 轴几乎朝上，
深度沿该轴积分把桌面点放到相机上方（z≈+0.59 m）。用总线实测做 FK，相机光轴
为 `(+0.51, -0.01, -0.86)`，即前下方，与桌面几何一致。厂商 `joint_state_publisher`
与控制器回显都不是编码器反馈；这是本轮最关键的发现。

### 5.2 实现

- 新节点 `jcheng_perception_edge/wrist_tf`：
  - 10 Hz 调用 `/ros_robot_controller/bus_servo/get_state`（5 个关节，1 次请求）；
  - 用实测脉冲换算关节角（joint3 反装）；
  - 解析实时 `/robot_description`，FK 求 `base_link→link4`；
  - 组合已验证 `T_link4_camera`；
  - 发布唯一动态帧 `camera_optical_measured`（父 `base_link`），避免与厂商
    `camera_optical_calibrated` 静态链冲突。
- 配置 `config/wrist_tf.yaml`；`config/cameras.yaml` 腕部 `optical_frame` 改为
  `camera_optical_measured`；缺失 TF 时丢观测（fail closed）。
- `pi5_edge.launch.py` 默认启动 `wrist_tf`（`start_wrist_tf:=false` 可关闭）。
- 运行时依赖厂商 `ros_robot_controller_msgs`；启动前需 source 厂商工作区。
- 5 关节读取实测 median 34.3 ms / max 39.1 ms；10 Hz 下 1000+ 次读取 0 失败。

### 5.3 单姿态验证（静止，无运动）

```text
TF: base_link -> camera_optical_measured
translation = (0.1753, 0.0082, 0.1973) m
view_z      = (0.5124, -0.0073, -0.8587)   # 前下方
depth plane fit: 4676 samples, inliers 1.000, RMS 0.61 mm
plane normal      = (-0.0162, -0.0136, 0.9998)
plane z at base origin = -0.0836 m
plane tilt        = 1.21 deg
camera above plane = 0.2809 m
reference table z = -0.0825 m  ->  difference 1.1 mm
```

深度用深度相机内参反投影，位姿用 RGB 手眼光学帧；IR-RGB 约 9.9 mm 基线未建模，
因此这是**单姿态**的平面一致性检查，不是多姿态绝对精度。下一步需在多个关节姿态
和固定目标上复核 x/y/z，之后才允许进入规划/运动。

## 6. DDS/QoS 发现

- 厂商 `CameraInfo`（Aurora）是 best-effort/volatile；edge 原来用 reliable +
  transient-local 订阅，`DURABILITY` 不兼容，收不到消息。已改为 sensor QoS
  （best-effort/volatile），配置内参作为回退。
- Fast DDS 与厂商 CycloneDDS 在域 0 混用时出现 `ParticipantEntitiesInfo` CDR
  反序列化错误；跨 RMW 不稳定。实机验证应在两端使用**同一 RMW**（推荐 CycloneDDS）。
  本次最终采用 Pi 5 本地 CycloneDDS。
- `tf2_echo` 看不到静态帧是其 `/tf_static` 订阅 QoS 限制，不代表 TF 树缺失。

## 7. 跨机真实 YOLO（外机位，检测链路；2026-11-24）

笔记本安装 `ros-jazzy-rmw-cyclonedds-cpp` 后，两端统一 CycloneDDS 域 0、有线直连，
首次跑通“Pi 5 edge → 笔记本 GPU”的真实模型检测链路：

```text
Edge: 外机位 UVC 彩色 1280x720，uplink 3 fps / q55，哑元腕部；限时 60 s
GPU:  RTX 3060，厂商 yolov8s.pt，imgsz 640，conf 0.20
```

```text
GPU: fps 2.06, inference 55.2 ms 平均, frames 37, dropped 4, failed 0
Edge: camera_fps 4.39, frames 115, detections 16, matched 0, dropped 16,
      depth_failures 25, latency 186.8 ms, gpu_alive true
检测: dining table 0.230, 1280x720, header.stamp 与 frame_seq 均为原采集时间
```

`matched=0` 是预期：彩色 1280x720 UVC 与深度 640x480 RealSense 未对齐，edge 在深度
匹配阶段丢弃；本测试只验证真实模型、跨机传输、采集时间戳回传与 GPU 健康。度量融合
仍需 ADR 0004。另一个信号：edge 的 `camera_fps` 只有 4.39，而原始彩色流是 14.36 Hz；
720p 全量 `cv_bridge` 转换已让 Pi 5 饱和。切换单一 RealSense 后应把彩色降到 640x480
或在 edge 端降采样，再谈 10 Hz 融合。

操作注意：

- 第一次**无界**尝试恰逢 Pi 应用层无响应（ping 正常，SSH banner/网关超时），随后
  断电重启。该命令还包含会匹配自身 shell 的 `pkill`，所以 edge 是否真正启动无法
  证实；之后用 `timeout -s INT 60` 的有界重试成功。结论：720p edge 在负载较高的
  Pi 上必须限时/监控，不能无界运行。
- Humble 与 Jazzy 的 CycloneDDS 组合记录了 `invalid data size` 序列化告警，但
  检测/状态流正确；生产环境应优先统一 ROS 发行版。
- 测试后 edge/GPU 进程为 0，四个厂商服务仍 active，负载回落到约 5.8，未发送运动命令。

## 8. 下一步（需要受监督窗口）

1. 已完成：腕部度量 TF 修正与单姿态桌面验证；仍需多姿态/固定目标复核 x/y/z。
2. 按 ADR 0004 切换 D435i 为单一 RealSense 所有者 + `align_depth_to_color`，
   退役 UVC 彩色路径；验证 Studio 预览仍从 ROS 彩色话题工作。
3. 外机位真实 YOLO 检测链路已通；仍待腕部真实模型/关键点、同发行版跨机联调。
4. 只有在 `jcheng_safety`/`jcheng_motion` 之下、且位姿经过固定目标核验后，才允许
   接入运动；本仓库不发送运动命令。

证据 JSON：
[`evidence/hardware_20261123/summary.json`](../evidence/hardware_20261123/summary.json)（首次接入）、
[`evidence/hardware_20261124/summary.json`](../evidence/hardware_20261124/summary.json)（TF 修正）、
[`evidence/hardware_20261124_yolo/summary.json`](../evidence/hardware_20261124_yolo/summary.json)（跨机真实 YOLO）。

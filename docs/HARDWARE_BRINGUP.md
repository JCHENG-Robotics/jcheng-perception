# 实机接入记录（只读/相机，2026-11-23）

> 本次没有发送任何运动命令，没有修改厂商容器或 systemd 服务；只启动了相机/感知进程并在结束后停止。
> 结论：**Pi 5 ARM64 构建与合成链路通过；腕部相机数据链路通过；但实机目标位姿的 z 不正确，
> 不能用于运动。** 外机位深度仍未对齐彩色。

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
0 失败。**但 z=+0.59 m 明显高于桌面（约 -0.08 m），说明当前 `base_link →
camera_optical_calibrated` 的组合在实机姿态下不成立。** 可能原因：

1. `robot_state_publisher` 的 `/joint_states` 是命令缓存而非总线实测；M4 审计已发现
   旧网关的 `measured` 与总线最大差 387 pulses。用这个 TF 计算相机位姿会产生大误差。
2. 腕部光学帧 `camera_optical_calibrated` 是项目侧静态帧；本次用 measured
   `rgb_camera_link → optical` 作为子帧，父链仍来自厂商 TF。
3. 深度与彩色是否逐像素对齐需要独立验证。

在修正前，**不得**把该位姿送入规划或运动。需要用总线实测关节 + URDF FK + 已验证的
`T_link4_camera` 重建 TF，并用桌面平面/固定标定板核验。

## 5. DDS/QoS 发现

- 厂商 `CameraInfo`（Aurora）是 best-effort/volatile；edge 原来用 reliable +
  transient-local 订阅，`DURABILITY` 不兼容，收不到消息。已改为 sensor QoS
  （best-effort/volatile），配置内参作为回退。
- Fast DDS 与厂商 CycloneDDS 在域 0 混用时出现 `ParticipantEntitiesInfo` CDR
  反序列化错误；跨 RMW 不稳定。实机验证应在两端使用**同一 RMW**（推荐 CycloneDDS）。
  本次最终采用 Pi 5 本地 CycloneDDS。
- `tf2_echo` 看不到静态帧是其 `/tf_static` 订阅 QoS 限制，不代表 TF 树缺失。

## 6. 下一步（需要受监督窗口）

1. 用总线实测关节/TF 修正腕部位姿；在桌面上放固定标定板，核验 `base_link` 下的
   平面 z 与目标 x/y。
2. 按 ADR 0004 切换 D435i 为单一 RealSense 所有者 + `align_depth_to_color`，
   退役 UVC 彩色路径；验证 Studio 预览仍从 ROS 彩色话题工作。
3. 用真实 YOLO（Ultralytics CUDA）替换蓝色 mock，在腕部/外机位分别验证。
4. 只有在 `jcheng_safety`/`jcheng_motion` 之下、且位姿经过固定目标核验后，才允许
   接入运动；本仓库不发送运动命令。

证据 JSON：[`evidence/hardware_20261123/summary.json`](../evidence/hardware_20261123/summary.json)。

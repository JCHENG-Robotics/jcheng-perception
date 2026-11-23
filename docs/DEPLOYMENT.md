# 部署：Pi 5 edge + 笔记本 GPU

> 本仓库只在 x86 Jazzy 主机上完成构建与合成端到端验证；以下实机步骤需要在
> ArmPi Ultra 的 Pi 5（Humble）和笔记本上执行并记录证据。不要直接把合成结果
> 当作硬件验证。

## 1. 前置条件

### Pi 5（机器人）

- ROS 2 Humble（厂商 `ArmPiUltra` 容器或主机环境）。
- 相机驱动：
  - 腕部 Aurora 930：厂商 `deptrum-ros-driver-aurora930` /
    `bringup vision.launch.py`；
  - 车头 D435i：**推荐 `realsense2_camera` 单所有者 + 彩色对齐深度**
    （见 [ADR 0004](adr/0004-camera-device-ownership.md)）。
- **当前部署的分离模式不适用于度量融合**：`armpi-ext-cam.service` 用 UVC
  打开 `/dev/video4` 发布 `/ext_cam/color/image_raw`，`armpi-rs-depth.service`
  用 `realsense2_camera` 只发布 `/camera/camera/depth/image_rect_raw`
  （`camera_depth_optical_frame`）。彩色与深度未对齐、无同源时间戳，不能在
  同一像素上做反投影。必须先按 ADR 0004 切换为
  `realsense2_camera` 彩色 + `aligned_depth_to_color`，再启用本仓库默认配置。
- Python：`numpy`、`PyYAML`、`opencv-python`/`python3-opencv`、`cv_bridge`、
  `tf2_ros`。
- **单设备所有者**：同一时刻只能有一个进程打开 D435i。推荐 RealSense 节点
  独占；Studio 网关改为订阅 `/camera/color/image_raw`，不再打开 `/dev/video4`。

### 笔记本 GPU

- ROS 2 Humble（与机器人一致最稳）或 Jazzy（本仓库构建验证版本）。
- Ultralytics（含 OpenVINO 导出支持）或 ONNX/OpenVINO 运行时。
- 模型权重放在仓库外，例如 `/opt/jcheng/models/`。
- 安装后先做无 ROS 烟测：

  ```bash
  python3 -c "import ultralytics, torch; print(ultralytics.__version__, torch.__version__)"
  python3 scripts/check_gpu_backend.py \
    --backend ultralytics --model /opt/jcheng/models/ext_yolov8n.pt \
    --image /data/sample.jpg
  ```

  当前这台主机没有 Ultralytics/torch/OpenVINO，真实后端烟测尚未执行；不要在没有
  烟测通过的情况下直接启动生产检测节点。
- 候选模型先在影子节点跑（`shadow_gpu.launch.py`），确认无误再切换生产话题。

## 2. 构建

```bash
cd jcheng-perception/ros2_ws
source /opt/ros/humble/setup.bash          # 机器人；笔记本用对应发行版
colcon build --packages-select \
  jcheng_perception_msgs jcheng_perception_core \
  jcheng_perception_edge jcheng_perception_gpu
source install/setup.bash
```

配置通过包内 `share/<pkg>/config/` 安装（源仓库的 `config/` 通过符号链接共享）。
也可在 launch 参数里覆盖为任意绝对路径。

## 3. 相机检查（不动机械臂）

```bash
# RGB / 深度 / 内参（推荐：RealSense 单所有者 + 对齐深度）
ros2 topic hz /depth_cam/rgb/image_raw
ros2 topic hz /depth_cam/depth/image_raw
ros2 topic echo --once /depth_cam/rgb/camera_info | head

ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw

# TF：两个光学帧都必须存在
ros2 run tf2_ros tf2_echo base_link camera_optical_calibrated
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

> 若现场仍是分离的 `/ext_cam/color/image_raw` +
> `/camera/camera/depth/image_rect_raw`，只能用于预览/调试，不能接入本仓库的
> 度量融合。先按 [ADR 0004](adr/0004-camera-device-ownership.md) 统一所有者并
> 开启对齐深度。

## 4. 启动

### 4.1 Pi 5

```bash
export ROS_DOMAIN_ID=0                 # 与厂商栈同域；或按系统方案用 42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # 机器人推荐（已装时）
ros2 launch jcheng_perception_edge pi5_edge.launch.py
```

可选参数：

```bash
ros2 launch jcheng_perception_edge pi5_edge.launch.py \
  target_frame:=base_link \
  cameras_config:=/abs/path/cameras.yaml \
  fusion_config:=/abs/path/fusion.yaml \
  frames_config:=/abs/path/frames.yaml \
  timing_log:=true
```

`timing_log:=true` 每秒输出各回调均值/最大值与 pending 数；用于现场调优，
正常运行时关闭。

### 4.2 笔记本 GPU

```bash
export ROS_DOMAIN_ID=0                 # 必须与 Pi 5 相同
ros2 launch jcheng_perception_gpu laptop_gpu.launch.py
```

模型路径在 `config/gpu.yaml`；若模型尚未就绪，节点会每秒重试加载并输出错误，
不会崩溃。热加载：替换权重文件后，worker 检测 mtime 并在下一帧前重新加载。

### 4.3 建议的域与网络

- 有线 eth0 直连（机器人 10.42.0.218，笔记本 10.42.0.1）。
- 两端使用同一 `ROS_DOMAIN_ID` 与同一 RMW；CycloneDDS 配置绑定 eth0，避免
  WiFi 多播噪声（参考 `jcheng-manipulation/docs/hardware/ros_environment.md`）。
- 测试/演示建议用独立域（如 `ROS_DOMAIN_ID=77`）隔离旧的 DDS 参与者。
- 状态流是 best-effort；不要为了“可靠”改回 reliable——慢订阅者会反压 edge。

## 5. 验证清单

```bash
# 1) 上行存在且带时间戳
ros2 topic hz --qos-reliability best_effort \
  /perception/uplink/aurora_wrist/image_compressed

# 2) GPU 检测存在
ros2 topic hz --qos-reliability best_effort \
  /perception/detections/aurora_wrist

# 3) 融合目标
ros2 topic echo --once --qos-reliability best_effort /perception/targets

# 4) 健康
ros2 topic echo --once --qos-reliability best_effort /perception/status
ros2 topic echo --once --qos-reliability best_effort /perception/gpu_status

# 5) 服务
ros2 service call /jcheng_perception_edge/drop_recheck std_srvs/srv/Trigger
ros2 service call /jcheng_perception_edge/reset std_srvs/srv/Trigger
```

通过标准（合成/离线）：

- `camera_fps` 接近相机实际帧率；
- `match_success_rate > 0.95`（排除启动前 2 s）；
- `/perception/targets` 中同一目标只有 **1 条** confirmed 轨迹；
- 固定目标时外机位与腕相机位置差在标定误差量级（合成场景 ~毫米）；
- `drop_recheck` 后权重为 external 1.0、wrist 0.0，`recheck_required=true`。

实机还需要：固定目标静止误差、移动目标跟踪误差、遮挡恢复时间、30 分钟
长时间稳定性、GPU 断线降级行为。上述均**尚未执行**。

## 6. 带宽与资源估算

| 流 | 分辨率 | 帧率 | 估计带宽（JPEG） |
|---|---|---|---|
| D435i 上行 | 1280×720 | 15 Hz | ~1.0–1.8 MB/s（q75，实际场景） |
| Aurora 上行 | 640×400 | 12 Hz | ~0.2–0.5 MB/s |
| 检测/观测/目标 | 小消息 | 10–30 Hz | < 0.1 MB/s |

千兆有线足够。WiFi 下建议降低 `max_uplink_fps`、`jpeg_quality` 或分辨率。
Pi 5 上 JPEG 编码在独立线程，主要占用 CPU；实测合成场景 edge 进程约 25% 单核。

## 7. systemd（可选）

```ini
# /etc/systemd/system/jcheng-perception-edge.service
[Unit]
Description=JCHENG perception edge
After=network-online.target

[Service]
User=ubuntu
Environment=ROS_DOMAIN_ID=0
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source /home/ubuntu/jcheng-perception/ros2_ws/install/setup.bash && exec ros2 launch jcheng_perception_edge pi5_edge.launch.py'
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

> 机器人厂商栈在容器内运行时，unit 需要进入对应容器/命名空间；不要和厂商
> `start_node.service` 争抢设备或域。

## 8. 故障排查

| 现象 | 优先检查 |
|---|---|
| 无 `/perception/targets` | edge 是否启动、`ROS_DOMAIN_ID`/RMW 是否一致、QoS 是否 best-effort |
| `camera_fps` 低 | 相机驱动帧率、CPU、`timing_log` 各回调耗时、是否有重复节点 |
| 匹配率低 | 深度/彩色时间戳是否同源、容差、TF 是否存在、分辨率是否一致 |
| 检测延迟高 | GPU 队列 `frames_dropped`、模型耗时、网络、是否 reliable 回压 |
| 同一目标两条轨迹 | 检查合并参数、标签是否一致、两相机位置偏差是否 >6 cm |
| 权重全给外机位 | 相位是否 FINE、腕相机深度是否在量程、TF 是否缺失、`keypoint_axis_pairs` |
| 模型未加载 | `gpu.yaml` 路径、Ultralytics 安装、权限；日志会每秒重试 |
| 运行数秒后整体卡顿 | 是否误用 reliable 状态流/多线程执行器；本版本 edge 固定单线程执行器 |

## 9. 安全边界

- edge 不发布运动命令；GPU 不接触相机/深度/TF。
- 空目标数组 = 当前无目标；规划器应停止接近或保持安全距离，不要“沿用上一条”。
- 目标位姿是建议值，规划器仍需工作空间/限位/碰撞校验。
- 软件停止不是急停；本平台没有急停/抱闸。

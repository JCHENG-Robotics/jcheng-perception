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
- **ADR 0004 已在实机执行（2026-11-24）**：`armpi-ext-cam.service` 已 disable，
  `armpi-rs-depth.service` 的 `rs_launch.py` 现在同时开彩色与对齐深度：

  ```text
  depth_module.depth_profile:=640x480x15
  rgb_camera.color_profile:=640x480x15
  enable_color:=true
  align_depth.enable:=true
  camera_namespace:=/
  ```

  `camera_namespace` 必须是 `/`：默认 `camera` 会发布 `/camera/camera/*`，
  与本仓库默认配置不一致；空值（`camera_namespace:=`）会被 `ros2 launch`
  判为非法参数并进入 systemd 重启循环。切换后实测彩色 6.9 Hz、对齐深度
  7.5 Hz、两路时间戳完全一致（见 `evidence/hardware_20261124_adr0004/`）。
- **回退到分离模式**（仅在 RealSense 彩色流异常时）：恢复
  `/home/pi/adr0004-20261124/` 下的备份 → 重启 `armpi-rs-depth` →
  `systemctl start armpi-ext-cam` → 去掉 gateway 的
  `ARMPI_GATEWAY_AUX_CAMERA_TOPIC`。分离模式只能预览/调试，不能做度量融合。
- Python：`numpy`、`PyYAML`、`opencv-python`/`python3-opencv`、`cv_bridge`、
  `tf2_ros`。
- 腕部 TF：`wrist_tf` 需要厂商 `ros_robot_controller_msgs` 和
  `/ros_robot_controller/bus_servo/get_state`；启动前 source 厂商工作区
  （容器内通常 `source ~/.zshrc`）。它用总线实测关节 + `/robot_description`
  FK + 手眼发布 `camera_optical_measured`；不要用命令缓存 `/joint_states` 的 TF。
- **单设备所有者**：同一时刻只能有一个进程打开 D435i。推荐 RealSense 节点
  独占；Studio 网关改为订阅 `/camera/color/image_raw`，不再打开 `/dev/video4`。

### 笔记本 GPU

- ROS 2 Humble（与机器人一致最稳）或 Jazzy（本仓库构建验证版本）。
- 推理运行时，按任务选择：
  - `ultralytics`（torch）：detect/pose/obb，GPU 主机首选；
  - `onnxruntime`：detect，CPU 或 ORT-GPU（本机已装 1.32.0）；
  - `openvino`：detect，Intel CPU/GPU，厂商 IR（本机已装 2026.4.1）；
  - `mock`：无模型演示/CI。
- 模型权重放在仓库外，例如 `/opt/jcheng/models/`。
- 推荐用独立 venv，避免污染 ROS 的 numpy/OpenCV：

  ```bash
  RUNTIME=onnxruntime ./scripts/setup_gpu_venv.sh
  RUNTIME=openvino    ./scripts/setup_gpu_venv.sh
  # GPU 主机、网络允许时：
  RUNTIME=ultralytics ./scripts/setup_gpu_venv.sh
  ```

- Docker 替代方案：`deploy/Dockerfile.gpu` 把同一套运行时（torch
  2.9.1+cu128 / ultralytics 8.4.216 / onnxruntime **1.23.2** / openvino
  2026.4.1；1.23.2 是 Python 3.10 可用的最高版本，宿主 venv 的 1.32.0 要求
  Python ≥3.11）装进 Humble 镜像，节点与机器人回到同发行版；`--gpus all`
  的宿主机前置步骤、构建和验证命令见 [`deploy/README.md`](../deploy/README.md)。
  镜像已于 2026-11-25 构建并验证（14.5 GB；RTX 3060 在容器内可见，
  `cuda_available()` 为 true，三个感知包可在 `/workspace/tag_ws` 构建）；
  同日完成真机 smoke：tag 后端 2.94/2.69 ms、edge `gpu_alive=true`，stock
  `yolov8n.pt` 稳态 8.46/8.80 ms（容器内 `nvidia-smi` 看到 210 MiB 的
  节点进程），证据在 `evidence/hardware_20261125_gpu_image/`。生产用的
  `ext_yolov8n.pt` / `wrist_grasp_pose.pt` 仍未就位，因此 YOLO 只验证了
  镜像与后端，不代表检测精度。

- 安装后先做无 ROS 烟测：

  ```bash
  # ONNX Runtime（本机实测：yolov8s.onnx，4 框，197.5 ms）
  ~/.venvs/jcheng-perception-gpu/bin/python scripts/check_gpu_backend.py \
    --backend onnxruntime --model /path/to/yolov8s.onnx \
    --image /path/to/sample.jpg

  # OpenVINO（本机实测：yolov8s.xml，4 框，126.7 ms）
  ~/.venvs/jcheng-perception-gpu/bin/python scripts/check_gpu_backend.py \
    --backend openvino --model /path/to/yolov8s.xml \
    --image /path/to/sample.jpg --device CPU

  # Ultralytics（CPU 或 GPU；本机 CPU 与 RTX 3060 均已实测）
  ~/.venvs/jcheng-perception-gpu/bin/python scripts/check_gpu_backend.py \
    --backend ultralytics --model /path/to/yolov8s.pt \
    --image /path/to/sample.jpg --device 0
  ```

  当前这台主机已安装 ONNX Runtime、OpenVINO、Ultralytics 8.4.216、
  PyTorch 2.9.1+cu128 与 torchvision 0.24.1+cu128；RTX 3060 可用，
  `yolov8s.pt` 稳态 9.0 ms（FP32）/ 7.4 ms（FP16）。没有烟测通过不要直接启动生产检测节点。
- 用 venv 解释器启动 ROS 节点（console script 的 shebang 仍指向系统 Python）：

  ```bash
  ros2 launch jcheng_perception_gpu laptop_gpu.launch.py \
    gpu_config:=/abs/path/gpu.yaml \
    python_executable:=$HOME/.venvs/jcheng-perception-gpu/bin/python
  ```

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
ros2 run tf2_ros tf2_echo base_link camera_optical_measured   # wrist_tf
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

`camera_optical_measured` 缺失时腕部观测会被丢弃（fail closed）。确认
`wrist_tf` 日志有 `wrist_tf reads=... failures=0`，且 z 轴朝下/前下方（桌面场景）。

> 若现场仍是分离的 `/ext_cam/color/image_raw` +
> `/camera/camera/depth/image_rect_raw`（例如换了一张没跑过 ADR 0004 的 SD 卡），
> 只能用于预览/调试，不能接入本仓库的度量融合；按
> [ADR 0004](adr/0004-camera-device-ownership.md) 统一所有者并开启对齐深度。
> `ext_tf`（随 `pi5_edge.launch.py` 默认启动）从 `frames.yaml` 发布
> `base_link → camera_link`，因此 `base_link → camera_color_optical_frame`
> 应当能直接解析；若不行，先确认 RealSense 节点在发 `camera_link`。

> **Aurora 深度退化（2026-11-25 实测）**：驱动连续运行 12 h 后，ingress 在
> 22 s 内收到 RGB 175 帧、深度只有 58 帧（约 7.95 / 2.64 Hz），融合观测率
> 降到 0.6–1.5 fps；`sudo systemctl restart armpi-vision`（oneshot 就绪轮询
> 约 2.5 min）后实测 **RGB 10.92 / 深度 10.83 Hz**。每次会话前跑
> `scripts/check_camera_rates.py`（阈值 RGB 6 / depth 5 Hz）；低帧率就重启
> 驱动，ingress 看门狗会自动重启其进程。证据：
> `evidence/hardware_20261125_gpu_image/camera_rates_after_restart.txt`。

## 4. 启动

### 4.1 Pi 5

```bash
export ROS_DOMAIN_ID=0                 # 与厂商栈同域；或按系统方案用 42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # 机器人推荐（已装时）
ros2 launch jcheng_perception_edge pi5_edge.launch.py
```

`pi5_edge.launch.py` 默认同时启动 `wrist_tf`。若已单独启动，传
`start_wrist_tf:=false` 避免重复。单独启动：

```bash
ros2 launch jcheng_perception_edge wrist_tf.launch.py
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

### 4.4 跨机 CycloneDDS（2026-11-24 已验证）

笔记本安装与机器人相同的 RMW：

```bash
sudo apt-get install -y ros-jazzy-rmw-cyclonedds-cpp
# 本次实测 cyclonedds 0.10.5 / rmw_cyclonedds_cpp 2.2.4
```

两端都绑定有线接口。机器人已有 `/home/ubuntu/cyclonedds.xml`（`eth0`）；
笔记本用 USB 网卡名，例如：

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

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///tmp/cyclonedds-laptop.xml
export ROS2CLI_NO_DAEMON=1
ros2 topic list          # 应看到机器人话题
ros2 topic echo /joint_states --once --field name
```

Humble（机器人）与 Jazzy（笔记本）跨发行版运行时，CycloneDDS 可能记录
`invalid data size` 序列化告警；本次检测/状态流仍正确，但生产环境应优先统一
ROS 发行版。跨机真实 YOLO 的实测结果见
[`EVIDENCE.md`](EVIDENCE.md) 2.10。

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

# 6) 腕部实测 TF（应 10 Hz 更新，z 轴朝前下方）
ros2 run tf2_ros tf2_echo base_link camera_optical_measured
```

通过标准（合成/离线）：

- `camera_fps` 接近相机实际帧率；
- `match_success_rate > 0.95`（排除启动前 2 s）；
- `/perception/targets` 中同一目标只有 **1 条** confirmed 轨迹；
- 固定目标时外机位与腕相机位置差在标定误差量级（合成场景 ~毫米）；
- `drop_recheck` 后权重为 external 1.0、wrist 0.0，`recheck_required=true`。

实机已完成：单静态姿态的腕部 TF 桌面平面核验（z=-0.0836 m，参考 -0.0825 m，
RMS 0.61 mm）。仍需：多姿态/固定目标 x/y/z、移动目标跟踪误差、遮挡恢复时间、
30 分钟长时间稳定性、GPU 断线降级行为。

## 6. 带宽与资源估算

| 流 | 分辨率 | 帧率 | 估计带宽（JPEG） |
|---|---|---|---|
| D435i 上行 | 1280×720 | 15 Hz | ~1.0–1.8 MB/s（q75，实际场景） |
| Aurora 上行 | 640×400 | 12 Hz | ~0.2–0.5 MB/s |
| 检测/观测/目标 | 小消息 | 10–30 Hz | < 0.1 MB/s |

千兆有线足够。WiFi 下建议降低 `max_uplink_fps`、`jpeg_quality` 或分辨率。
Pi 5 上 JPEG 编码在独立线程，主要占用 CPU；实测合成场景 edge 进程约 25% 单核。

## 7. systemd（split 三进程）

生产上用 systemd 管理 split 部署，不再手动 `ros2 launch`。每个进程一个 unit，
launch 的 `respawn=True` 由 systemd 接管：

| unit | 内容 |
|---|---|
| `jcheng-perception-ingress@d435i_ext.service` | 外机位 ingress |
| `jcheng-perception-ingress@aurora_wrist.service` | 腕部 ingress |
| `jcheng-perception-fusion.service` | 融合 |
| `jcheng-perception-wrist-tf.service` | 腕部动态 TF（总线舵机，只读） |
| `jcheng-perception-ext-tf.service` | 外机位静态链 `base_link→camera_link` |
| `jcheng-perception.target` | 以上五者一起启停、开机自启 |
| `jcheng-perception-alert@.service` | 超过重启上限时的告警 |

```bash
sudo bash deploy/systemd/install-split-systemd.sh --start
sudo systemctl stop jcheng-perception.target     # PartOf= 停掉五个进程
journalctl -u 'jcheng-perception-*' -f
```

要点：

- ingress watchdog 仍以 **0** 退出（为了换新 DDS participant），所以 unit 用
  `Restart=always` + `RestartSec=2`，不能用 `on-failure`；
- 每个 ingress 的重启上限是 **10 次 / 300 s**（`StartLimitIntervalSec`/
  `StartLimitBurst`）。相机彻底失效时约 40–60 s 后 unit 进入 `failed` 并触发
  `jcheng-perception-alert@ingress-<camera>`：写 journal `user.err`、
  `/var/lib/jcheng-perception/last-alert.txt` 与 `alerts.log`（无外网，不发远程
  通知）。修复后显式 `systemctl reset-failed && systemctl start`；
- 包装脚本在容器内运行，显式使用 CycloneDDS 和 domain 0，与相机驱动一致；
- 所有容器内 unit 都 `After=/Wants=armpi-vision.service`：`start_node` 启动时会
  执行 `~/.stop_ros.sh`，对命令行含 “ros” 的进程 `kill -9`；`armpi-vision`
  只在厂商 bringup 起来后 active，等待它可以避开这个窗口（2026-11-25 开机时
  `ext-tf` 就因只等 `armpi-rs-depth` 被 SIGKILL 137）；
- 两个 TF 发布器也在 target 内：`wrist_tf` 没有 `camera_optical_measured` 时腕部
  观测会被丢弃，`ext_tf` 让外机位链有 `base_link`。`wrist_tf` 只读总线舵机
  （`get_state`），不发运动命令，依赖厂商工作区的 `ros_robot_controller_msgs`
  （`perception.env` 的 `JCHENG_PERCEPTION_VENDOR_SETUP`）。
- 笔记本 GPU 节点是 user unit（`deploy/systemd/laptop/`）：
  `jcheng-perception-gpu.service` 用 `/opt/jcheng/models/yolov8s.pt` 和
  `gpu_laptop.yaml`，`Restart=always` + 10/300 s + 本地 OnFailure 告警。
  已知 reader wedge（进程活着但 `frames_received` 冻结）由
  `jcheng-perception-gpu-watchdog` 覆盖：单路 60 s 不增长/状态 30 s 缺失时
  重启，预算 5 次 / 30 min 后告警；两路都冻结视为 edge 掉线不重启。

完整安装/验证/告警/回滚说明见
[`deploy/systemd/README.md`](../deploy/systemd/README.md)。厂商栈在容器内运行，
unit 只通过 `docker exec` 进入容器，不打开相机设备、不与 `start_node.service`
争抢设备或域。

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

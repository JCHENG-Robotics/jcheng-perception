# 验证证据（2026-11-23）

> **范围声明**：以下全部是 **x86 工作站 + ROS 2 Jazzy + 合成场景** 的结果，外加
> 厂商 `yolov8s` 权重的**单图 CPU 推理**。**没有连接机器人、没有 Pi 5 实测。**
> 合成结果只证明数据流、时间匹配、融合逻辑与并发结构正确，不代表实机精度/延迟。

## 1. 环境

| 项 | 值 |
|---|---|
| 主机 | Lenovo Legion R9000P，Ubuntu 24.04，x86_64，16 CPU，RTX 3060 6 GB |
| ROS | Jazzy（`/opt/ros/jazzy`） |
| Python | 3.12.3 |
| RMW | Fast DDS（默认） |
| 测试域 | `ROS_DOMAIN_ID=77/78/79`（隔离旧 DDS 参与者） |
| 场景 | 640×400 @ 10 Hz，两路合成 RGB+深度，外机位用 2026-11-20 实测 `T_base_front`，桌面 z=-0.0825 m |
| 检测器 | `mock-color-red`；真实烟测用厂商 `yolov8s.onnx/.xml`（COCO，CPU） |
| 推理运行时 | venv `~/.venvs/jcheng-perception-gpu`：ONNX Runtime 1.32.0、OpenVINO 2026.4.1、numpy 1.26.4 |

## 2. 命令与结果

### 2.1 核心测试

```bash
python3 -m pytest tests -q
# 53 passed
```

覆盖：时间环/排序/剪枝、四元数/slerp、TF 插值、深度采样/空洞、反投影、
协方差、关键点轴、遮挡、关联/生命周期、双视图信息加权、相位、DROP_RECHECK、
重复轨迹合并、配置校验、edge/core 无推理代码。

### 2.2 离线全链路仿真

```bash
python3 scripts/simulate_pipeline.py
```

```text
frames: 100  target presence: 100.0%
external_only      n=100 rmse=0.59 mm p95=1.04 mm max=1.16 mm
wrist_only         n=100 rmse=0.54 mm p95=0.50 mm max=1.50 mm
fused              n= 85 rmse=0.57 mm p95=0.81 mm max=1.96 mm
occlusion_window   n= 10 rmse=0.76 mm p95=1.31 mm max=1.96 mm
phases: {'ACQUIRE': 2, 'FINE': 98}
drop recheck: {'phase': 'DROP_RECHECK', 'recheck_required': True, 'weights': {'d435i_ext': 1.0}}
```

- 外机位被遮挡 1 s 期间，融合目标仍存在，误差未发散。
- `DROP_RECHECK` 下腕相机权重被先验置 0，外机位权重 1.0。
- 这是**注入噪声的合成深度**，不是实机精度。

### 2.3 ROS 2 构建

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  jcheng_perception_msgs jcheng_perception_core \
  jcheng_perception_edge jcheng_perception_gpu
# Summary: 4 packages finished
```

### 2.4 单机合成端到端

```bash
export ROS_DOMAIN_ID=77
ros2 launch jcheng_perception_gpu synthetic_demo.launch.py
```

`/perception/status`（运行约 20 s）：

```text
camera_fps: 10.000 / 9.999
total_frames: 216
total_detections: 210
detections_matched: 208
detections_dropped: 0
match_success_rate: 0.9905
mean_detection_latency_ms: 26.6
phase: 3 (FINE)
gpu_alive: true
active_tracks: 1
confirmed_tracks: 1
notes: []
```

`/perception/targets`（最后一条消息）：

```text
target_id: target-001
position: (0.3535, -0.0041, -0.0805)
source_cameras: ['d435i_ext', 'aurora_wrist']
status: 1 (CONFIRMED)
phase: 3 (FINE)
weight_external: 0.21
weight_wrist: 0.79
```

edge 回调耗时（`timing_log:=true`）：

```text
depth=0.1-0.2 ms
detections=0.5-0.6 ms
image=0.3-0.4 ms
publish=1.1-1.3 ms (max ~4 ms)
tf=0.2-0.3 ms
pending=0-2
```

### 2.5 GPU 后端烟测

独立 venv（带 system site-packages，不污染 ROS）：

```bash
RUNTIME=onnxruntime ./scripts/setup_gpu_venv.sh
RUNTIME=openvino    ./scripts/setup_gpu_venv.sh
```

| 后端 | 模型 | 加载 | 单图推理 | 结果 |
|---|---|---|---|---|
| mock | `mock-color-red` | 0 ms | 6.2 ms | 1 个框 |
| ONNX Runtime 1.32.0 | 厂商 `yolov8s.onnx` | 171.4 ms | **197.5 ms** | 4 个框 |
| OpenVINO 2026.4.1 | 厂商 `yolov8s.xml` | 297.5 ms | **126.7 ms** | 4 个框，坐标一致 |

真实模型检测结果（COCO class id / score / xyxy）：

```text
49 0.933 [173.46, 262.31, 363.57, 440.13]
67 0.806 [0.36, 337.67, 72.04, 472.88]
39 0.762 [294.38, 50.54, 369.17, 186.58]
0  0.263 [5.79, 177.13, 639.0, 468.16]
```

ROS 节点内验证（ONNX Runtime，venv Python 通过 `python_executable`）：

```text
process: /home/.../.venvs/jcheng-perception-gpu/bin/python .../perception_gpu
model_ids: yolov8s.onnx:1775123732000000000
inference_ms: 232.8
fps: 3.35 / 3.61 (CPU，低于 10 Hz 输入；GPU 主机应使用 Ultralytics/CUDA)
frames_received: 53 / 57，dropped 0，failed 0
```

OpenCV 4.6 DNN 无法解析 YOLOv8 的 `Reshape/DFL` 节点，因此 `opencv` 后端仅保留
给较老模型；YOLOv8 请用 `onnxruntime`、`openvino` 或 `ultralytics`。

CUDA PyTorch/Ultralytics 未安装：CUDA 依赖约 1.5 GB，当前网络限速到不可接受。
GPU 主机网络允许时运行 `RUNTIME=ultralytics ./scripts/setup_gpu_venv.sh`。

### 2.6 影子模型隔离（mock）

在生产合成场景运行的同时启动 `shadow_gpu.launch.py`，影子配置使用 mock 检测器、
发布到 `*_shadow`：

```text
影子: /perception/gpu_status_shadow
      fps 9.48 / 9.48，frames_received 57，dropped 0
生产: camera_fps 10.00 / 10.00
      match_success_rate 1.0，active_tracks 1，confirmed_tracks 1，phase FINE
```

影子话题与生产话题完全分离；生产融合结果没有出现额外轨迹或相位变化。真实模型
的影子对比仍需 GPU 主机运行时。

## 3. 本轮发现并修复的问题

| # | 现象 | 根因 | 修复 |
|---|---|---|---|
| 1 | 首次运行 `ValueError: could not broadcast` | 6×6 协方差写入用了 `covariance[:3]` | 改为 `[:9]` |
| 2 | edge 单核 99%，上行仅 1 Hz | 100 Hz × 2 相机 TF 查询占满核 | 外机位 0.5 Hz、腕部 30 Hz |
| 3 | 上行/检测率约为相机一半 | 严格 `1/fps` 限速遇上 95–104 ms 抖动 | 15% 容差 |
| 4 | 编码/网络拖慢执行器 | JPEG 在 ROS 回调内 | 独立 uplink worker + depth-2 队列 |
| 5 | 运行数秒后所有回调卡数秒 | 每帧检测都发布/记录 status；rclpy 日志 `realpath` + reliable 回压 + 多线程执行器 | status 1 Hz；周期状态 best-effort；edge 单线程执行器 |
| 6 | 同一目标两条轨迹 | 两相机启动瞬态各建一条，之后各自匹配自己 | 重复轨迹合并（跨相机 6 cm/3σ，同相机严格 2 cm） |
| 7 | 真实后端烟测最初无法运行 | 主机无 torch/Ultralytics，CUDA 依赖下载限速 | 建独立 venv，安装 ONNX Runtime/OpenVINO CPU 并跑通真实 `yolov8s`；CUDA/Ultralytics 留待 GPU 主机 |
| 8 | 默认 D435i 配置与实机拓扑不符 | 实机是 UVC 彩色 + 仅深度 RealSense，深度未对齐彩色 | ADR 0004 提议单一 RealSense 所有者 + aligned depth；默认配置已注明 |

## 4. 未验证项

- Pi 5（ARM64、Humble）上的构建与运行；
- 真实 D435i/Aurora 驱动、USB 带宽、深度/彩色对齐；
- Ultralytics/CUDA 在 RTX 3060 上的推理延迟、关键点模型、热加载；
- 真实模型在 10 Hz 输入下的实时性（CPU 后端约 3.5–5 Hz）；
- 外机位单一所有者切换（RealSense 彩色 + aligned depth）与随之而来的深度帧率；
- 有线/无线网络下的端到端延迟与丢包；
- 实机标定误差、固定/移动目标精度、遮挡恢复时间；
- 30 分钟以上稳定性、CPU 温度/降频、内存；
- 与 `jcheng_system` 的域隔离/遥测镜像集成；
- 规划器消费 `FusedTargetArray` 的接口与安全校验。

## 5. 复现

```bash
git clone git@github.com:JCHENG-Robotics/jcheng-perception.git
cd jcheng-perception
python3 -m pytest tests -q
python3 scripts/simulate_pipeline.py
cd ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  jcheng_perception_msgs jcheng_perception_core \
  jcheng_perception_edge jcheng_perception_gpu
source install/setup.bash
ROS_DOMAIN_ID=77 ros2 launch jcheng_perception_gpu synthetic_demo.launch.py
```

配置/代码/结果如与本文件不一致，以仓库提交和重新运行结果为准。

# 验证证据（2026-11-23 → 2026-11-24）

> **范围声明**：主体是 **x86 工作站 + ROS 2 Jazzy + 合成场景**，外加厂商
> `yolov8s` 的单图 CPU/GPU 推理；**2026-11-23 做了 Pi 5 ARM64 合成与腕部真实相机
> 数据链路测试；2026-11-24 修正腕部度量 TF 并完成单静态姿态桌面平面核验，同时
> 跑通 Pi 5 edge → 笔记本 GPU 的外机位真实 YOLO 检测链路（无运动命令）**。
> 多姿态/固定目标精度、深度对齐与运动集成仍未验证，不能代表可用于运动的精度。

## 1. 环境

| 项 | 值 |
|---|---|
| 主机 | Lenovo Legion R9000P，Ubuntu 24.04，x86_64，16 CPU，RTX 3060 6 GB |
| ROS | Jazzy（`/opt/ros/jazzy`） |
| Python | 3.12.3 |
| RMW | Fast DDS（默认） |
| 测试域 | `ROS_DOMAIN_ID=77/78/79`（隔离旧 DDS 参与者） |
| 场景 | 640×400 @ 10 Hz，两路合成 RGB+深度，外机位用 2026-11-20 实测 `T_base_front`，桌面 z=-0.0825 m |
| 检测器 | `mock-color-red`；真实烟测用厂商 `yolov8s.onnx/.xml/.pt`、`yolov8n-obb.pt`（CPU 与 RTX 3060） |
| 推理运行时 | venv `~/.venvs/jcheng-perception-gpu`：ONNX Runtime 1.32.0、OpenVINO 2026.4.1、torch 2.9.1+cu128、torchvision 0.24.1+cu128、Ultralytics 8.4.216、numpy 1.26.4 |

## 2. 命令与结果

### 2.1 核心测试

```bash
python3 -m pytest tests -q
# 58 passed
```

覆盖：时间环/排序/剪枝、四元数/slerp、TF 插值、深度采样/空洞、反投影、
协方差、关键点轴、遮挡、关联/生命周期、双视图信息加权、相位、DROP_RECHECK、
重复轨迹合并、配置校验、腕部 URDF 链/FK/脉冲换算、edge/core 无推理代码。

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
RUNTIME=onnxruntime     ./scripts/setup_gpu_venv.sh
RUNTIME=openvino        ./scripts/setup_gpu_venv.sh
RUNTIME=ultralytics-cpu ./scripts/setup_gpu_venv.sh
```

| 后端 | 模型 | 加载 | 单图推理 | 结果 |
|---|---|---|---|---|
| mock | `mock-color-red` | 0 ms | 6.2 ms | 1 个框 |
| ONNX Runtime 1.32.0 | 厂商 `yolov8s.onnx` | 171.4 ms | **197.5 ms** | 4 个框 |
| OpenVINO 2026.4.1 | 厂商 `yolov8s.xml` | 297.5 ms | **126.7 ms** | 4 个框，坐标一致 |
| Ultralytics 8.4.216 (torch 2.6.0+cpu) | 厂商 `yolov8s.pt` | 1244.9 ms | **1039.8 ms** | 4 个框，标签正确 |
| Ultralytics (OBB) | 厂商 `yolov8n-obb.pt` | 1362.8 ms | **963.2 ms** | 1 个旋转框 |
| Ultralytics CUDA (torch 2.9.1+cu128) | 厂商 `yolov8s.pt` | 1397.3 ms | **2887.9 ms 首帧，9.0 ms FP32 / 7.4 ms FP16 稳态** | 4 个框，RTX 3060 |

真实模型检测结果（Ultralytics，标签）：`orange 0.941`、`cell phone 0.823`、
`bottle 0.762`、`person 0.268`。ONNX/OpenVINO 为同一组框的 class id。

ROS 节点内验证（ONNX Runtime，venv Python 通过 `python_executable`）：

```text
process: /home/.../.venvs/jcheng-perception-gpu/bin/python .../perception_gpu
model_ids: yolov8s.onnx:1775123732000000000
inference_ms: 232.8
fps: 3.35 / 3.61 (CPU，低于 10 Hz 输入)
frames_received: 53 / 57，dropped 0，failed 0
```

ROS 节点内验证（Ultralytics CPU）：

```text
model_ids: yolov8s.pt:1775123732000000000
inference_ms: 154.3 (steady state)
fps: 5.80 / 7.83
frames_received: 114 / 123，dropped 30 / 0，failed 0
```

ROS 节点内验证（Ultralytics CUDA / RTX 3060）：

```text
model_ids: yolov8s.pt:1775123732000000000
inference_ms: 54.27 (average, includes warm-up)
fps: 6.07 / 7.87
frames_received: 109 / 108，dropped 34 / 0，failed 0
nvidia-smi: 15% util, 412 MiB
```

OpenCV 4.6 DNN 无法解析 YOLOv8 的 `Reshape/DFL` 节点，因此 `opencv` 后端仅保留
给较老模型；YOLOv8 请用 `onnxruntime`、`openvino` 或 `ultralytics`。

CUDA 版已安装：`torch 2.9.1+cu128`、`torchvision 0.24.1+cu128`，RTX 3060 上
`torch.cuda.is_available()=True`，单图稳态 9.0 ms（FP32）/ 7.4 ms（FP16）。此前的失败是因为
torch 2.11 与按 2.9.1 元数据安装的 NCCL 版本不匹配（`ncclDevCommCreate`
未定义）；固定 2.9.1 组合后解决。安装脚本见 `RUNTIME=ultralytics-cuda`。
FP16 默认关闭（当前 Ultralytics 对 `half` 给出弃用警告），可按需开启。

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

### 2.7 Pi 5 ARM64 合成端到端

四个包在机器人容器（ROS 2 Humble）内构建通过（27.1 s），合成场景结果：

```text
camera_fps: 6.207 / 6.343
total_frames: 174, detections: 170, matched: 169, dropped: 0
match_success_rate: 0.9941
mean_detection_latency_ms: 48.09
phase: FINE, active_tracks: 1, confirmed_tracks: 1
weights: external 0.2203 / wrist 0.7797
```

### 2.8 腕部真实相机数据链路（部分）

Aurora 驱动从本地 SDK 重建后恢复：RGB 6.7–6.8 Hz、深度 3.1–6.4 Hz、640×400。
用蓝色方块 + mock 蓝色阈值检测器，edge/GPU 在 Pi 5 本地 CycloneDDS 域 0：

```text
GPU: fps 4.89, frames_received 39, failed 0, inference 3.22 ms
Edge: camera_fps 8.32, detections 47, matched 26, dropped 21 (depth_failures 20)
      match_success_rate 0.553, latency 71.46 ms, depth_age 34.5 ms
Target: blue_block-001, phase FINE, wrist weight 1.0, one confirmed track
Position: (0.1071, -0.0602, 0.5911) m
```

数据路径、时间戳回溯、深度/TF 匹配和融合都实际执行；当次 z=+0.59 m 明显高于
桌面，根因于 2026-11-24 定位为命令缓存 TF（见 2.9），数据路径本身有效。外机位仍是
未对齐的 UVC 彩色 + 仅深度 RealSense。详见
[HARDWARE_BRINGUP.md](HARDWARE_BRINGUP.md)。

### 2.9 腕部实测 TF 与桌面平面（2026-11-24）

同一静止姿态下，`/joint_states` 为 joint1=+0.8378、joint2–5=0；总线伺服为
joint1≈-0.013、joint2≈+0.406、joint3=-1.382、joint4=-1.629。用命令缓存组合
TF 会把相机 z 轴几乎指向正上方；`wrist_tf` 改用总线实测 + 实时
`/robot_description` FK + 已验证手眼，发布 `camera_optical_measured`：

```text
10 Hz, 1000+ reads, failures=0
5-joint bus read: median 34.3 ms, max 39.1 ms
base_link -> camera_optical_measured
translation = (0.1753, 0.0082, 0.1973) m
view_z      = (0.5124, -0.0073, -0.8587)   # 前下方
```

用深度相机内参反投影一帧深度，采样 4676 点拟合桌面：

```text
plane normal          = (-0.0162, -0.0136, 0.9998)
plane z at base origin = -0.0836 m
plane tilt             = 1.21 deg
plane RMS              = 0.61 mm
inlier fraction        = 1.000
camera above plane     = 0.2809 m
reference table z      = -0.0825 m
difference             = 1.1 mm
```

这是**单静态姿态**的平面一致性检查；深度用 RGB 光学位姿（IR-RGB 约 9.9 mm
基线未建模）。多姿态、固定目标 x/y/z 与运动中的 TF 滞后仍需验证。测试期间未发送
运动命令。

### 2.10 跨机真实 YOLO（外机位检测链路，2026-11-24）

笔记本安装 `ros-jazzy-rmw-cyclonedds-cpp` 后，两端统一 CycloneDDS 域 0、有线直连。
Pi 5 edge 订阅外机位 UVC 彩色（1280×720，uplink 3 fps / q55），笔记本 RTX 3060
运行厂商 `yolov8s.pt`（imgsz 640，conf 0.20）：

```text
GPU: fps 2.059, inference 55.2 ms 平均, frames_received 37, dropped 4, failed 0
Edge: camera_fps 4.387, total_frames 115, total_detections 16,
      matched 0, dropped 16, depth_failures 25, latency 186.8 ms, gpu_alive true
Detection: dining table 0.2297, 1280x720,
           header.stamp = frame_seq = 1795487500703345576 (原采集时间)
```

跨机发现验证：笔记本 `ros2 topic list` 可见机器人话题，`/joint_states` 可收。
`matched=0` 是预期：彩色 1280×720 UVC 与深度 640×480 RealSense 未对齐，edge 在
深度匹配阶段丢弃；本测试只验证真实模型、跨机传输、采集时间戳回传和 GPU 健康，
不代表度量融合可用。

操作注意：第一次**无界**尝试恰逢 Pi 应用层无响应（ping 正常，SSH banner/网关超时），
随后断电重启；该命令还包含会匹配自身 shell 的 `pkill`，所以 edge 是否真正启动无法
证实。之后用 `timeout -s INT 60` 有界重试成功；测试后进程为 0、四个厂商服务 active、
负载约 5.8。720p edge 在负载较高的 Pi 上必须限时/监控。

### 2.11 跨视角偏差建模与轨迹漂移回归（离线，2026-11-24）

不新增实机运行：用 `hardware_20261124_dualview/dual_verify7.json` 的 223 条观测
离线重放（测量协方差按 `cameras.yaml` 重建），复现 run-7 的静态目标漂移并验证
修复。模型见 `docs/FUSION.md` §9。

```bash
python3 -m pytest tests -q        # 65 passed（新增 7 条偏差/漂移测试）
python3 scripts/replay_static_drift.py \
  --out evidence/analysis_20261124_drift_bias/replay_fixed.json
python3 scripts/replay_static_drift.py --mode legacy \
  --out evidence/analysis_20261124_drift_bias/replay_legacy.json
python3 scripts/replay_static_drift.py --recorded-targets \
  --out evidence/analysis_20261124_drift_bias/recorded_targets.json
```

| 数据 | required 标签最大漂移 |
|---|---:|
| 实机录制发布值（修复前） | 696.4 mm（cup-005） |
| 离线重放，旧跟踪逻辑 | 477.8 mm |
| 离线重放，偏差模型 + 同视图速度 | **6.4 mm**（cup 4.1 / table 4.9 / chair 6.4 / person 4.4） |

机制：两个视图各自的测量很稳（cup 外部 σ 0.2 mm、腕部 0.4–1.1 mm），但框中心
落点不同造成 67/13/25 mm 的系统差；旧滤波器把它当速度（7–14 m/s），再按
`now - last_state` 外推最多 1 s，于是 0.5 m 漂移。修复后每 track 学到的残差与
离线统计吻合（cup −57.2/+10.9/+34.0 mm，table +22.1/+21.1/−5.0 mm）。
`kite` 未纳入门限：同一标签在两处物理位置间切换，属于关联/数据问题。
证据：`evidence/analysis_20261124_drift_bias/`。

### 2.12 偏差模型实机静置验证（2026-11-24）

把 `eed1ca9` 部署到 Pi 5 容器（`colcon build` 3 包 10.3 s），用
`fusion_dual_hardware.yaml` + `view_bias.yaml` 跑两段有界静置采集
（`timeout -s INT`，无运动命令；笔记本记录 observations/targets/status）：

```text
run 1  90 s  edge 窗口，实际有效观测 7.7–23.7 s，125 obs（外 100 / 腕 25），81 帧
run 2  60 s  完整，306 obs（外 295 / 腕 11），357 帧 / 1060 detections / 306 matched
```

静置指标（6 s 稳定窗口后的中位数偏差）：

| run | cup | dining table | kite | 最差 required |
|---|---:|---:|---:|---:|
| 1 | 2.73 mm | 4.63 mm | 0.16 mm | 4.65 mm |
| 2 | 0.31 mm | 8.13 mm | 1.61 mm | 8.13 mm |

离线重放与实机发布值最差相差 0.02 mm（run 2 完全一致），说明模型重放可信。
实机跨视角差（已扣链级修正）：cup (−56.9, +11.6, +34.3) mm、餐桌
(+29.2, +21.2, −3.9) mm（run 1），与 run-7 记录一致。

发现：

- **首个跨视图配对会产生一次参考点跳变**：run 1 cup 在 t=12.32 s 从外机位
  参考点 (335.9, −19.5, −29.7) mm 跳到腕部参考点 (392.8, −31.6, −63.6) mm，
  67.2 mm；这是 `Track.view_offset` 初始化，不是漂移。规划器要么容忍这次
  参考切换，要么在腕部参考建立前不发布该轨迹（未实现，属策略决定）。
- **run 1 的 edge 读端在 ~23.7 s 后不再收帧**，但 edge 的 status 继续到超时，
  相机驱动在跑完后仍以 6.95/4.4 Hz 发布，无 USB/xhci 错误；run 2 完整 60 s，
  未复现。更像已知的 Humble↔Jazzy CycloneDDS 读端 wedge，需单独处理。
- **run 2 腕部深度匹配失败 172/357**，只有 11 条腕部观测，稳定窗口内没有
  跨视图配对；run 2 的轨迹基本是单视图，8.1 mm 是餐桌框的检测抖动，不是 TF。
- 绝对位置仍是各视图的框中心点：cup 外机位/腕部轨迹相距 57 mm，不能把它
  当成物体中心精度；<5 mm 绝对精度仍需固定点/键点。
- 容器内存 1.4/6 GB；dmesg 有一次更早（16:04）的 memcg OOM 和 18:16–18:18
  的 `__vm_enough_memory`（PID 已退出），本次运行窗口内无新 OOM，未根因。

证据：`evidence/hardware_20261124_drift_verify/`。

### 2.13 固定点（AprilTag）参考点验证（2026-11-24）

GPU 新增 `aruco` 后端（`config/gpu_tag.yaml`，36h11、60 mm、id 0），标签中心
作为关键点；edge 不改。实机共三次尝试，前两次分别是 edge 在 import 阶段被
60 s timeout 打断（0 观测）和 laptop GPU 外机位 uplink 读端 wedge（仅腕部
30 观测）；第三次用“先启动 edge、等 ready，再重启 GPU 节点”的顺序恢复双视图：

```text
run 3: 40 s, 311 observations (120 external / 191 wrist), 0.2–40.1 s
per-view tag centre: ext (330.5, -14.3, -82.8) mm, wrist (330.3, -12.3, -79.6) mm
ext - wrist = (+0.2, -2.0, -3.2) mm -> 3.77 mm
learned Track.view_offset = (+0.03, -1.93, -3.31) mm
```

`fixed_point_check.py` 每视图 20 s（36/30 样本，tf=exact，重投影 0.21/0.75 px），
5 mm gate：

| 比较 | 原始 | 加 `view_bias.yaml`（外机位 +9.9/−1.8/+9.4 mm）|
|---|---:|---:|
| depth（wrist − ext） | 15.92 mm FAIL | **3.67 mm PASS** |
| RGB PnP（wrist − ext） | 12.79 mm FAIL | **3.07 mm PASS** |

- 没有 67 mm 首配对跳变：融合轨迹从外机位点起步，只按 3.8 mm 残差切到腕部
  参考；实机最大 11.4 mm / p95 6.2 mm / 最终 3.7 mm（eed1ca9）。
- 标签数据暴露近重复 capture 导致的伪速度：47/191 条腕部观测 dt≤0，1 mm 差
  除以 0.04 s = 25 mm/s。新增 `velocity_min_dt_s=0.10`；重放降到 8.3 mm /
  p95 5.4 mm，并让 run1/2 的离线重放与实机录制重新对齐（0.0 / 0.22 mm）。
  该修正在工作区，尚未部署到 Pi。
- 绝对参考：腕部 PnP (335.4, −12.7, −85.7) mm；修正后外机位 PnP
  (334.7, −14.7, −83.5) mm。每视图 depth-vs-PnP 仍差 4.2/7.4 mm（标签纸翘曲、
  斜视），要 <5 mm 绝对精度需先压平标签。
- 部署 `f90b5d5`（min-dt）后复跑：只得到外机位 45 条 tag 观测（0.3–14.3 s），
  腕部 depth failures 365、GPU 外机位流 ~14 s 后停；融合轨迹 6 s 后 0.4 mm
  （max 2.42 mm），min-dt 无漂移，但**未取得双视图复跑**。
- 第二次复跑（重启 Aurora 驱动 + GPU 每 10 s 重启）：腕部 depth failures
  365→**1**，但 edge 约 6 s 后帧摄入停止（88 帧、27 detections、gpu_alive=false），
  外机位-only 轨迹 1.8 mm；双视图仍未取得。
- Humble 同发行版复跑（run 6）：新建 `jcheng-humble-tag` 容器（`armpi-humble-moveit`
  + `python3-opencv` 4.5.4 + msgs/core/gpu 在 `/workspace/tag_ws` 构建），laptop
  GPU 与 Pi edge 同为 Humble；并修复 edge uplink 时钟回跳基线
  (`uplink_allowed`) 与 publish 异常导致线程退出。60 s：141 条腕部 tag 观测
  （0.5–59.8 s）、edge 468 帧/234 matched；**腕部-only** 轨迹 max **7.30 mm**、
  p95 4.66、final 3.93 mm。外机位仍 0 条 tag 观测（GPU 只收到 42 外机位帧，
  12 s 检查无外机位 uplink），当前外机位画面和 0.225 m 深度正常，剩余阻塞是
  外机位 uplink/reader。
- 订阅 watchdog 诊断（已回退）：外机位 RGB 卡在 5 帧（depth 仍增长），腕部
  depth 卡在 7（RGB 仍增长）；重建订阅 3 次不恢复，说明 wedge 在 DDS
  participant/reader 层。watchdog 未提交，Pi 运行 committed `2a33cbb`。
  双视图结果仍以上述 run 3 为准。
- **网关根因（run 7–9）：** aux 网关 8765 未传 RMW，默认 Fast DDS 订阅外机位
  彩色，跨 RMW 导致外机位 RGB reader wedge；改为 CycloneDDS 后外机位连续 60 s。
  8766 motion 网关是一个 PPID=0 的孤儿进程，仍订阅腕部点云、阻塞 Aurora
  depth 发布；kill 后腕部 depth 恢复。
- **Run 9 双视图成功：** external 240 + wrist 195 条（0.1–60.1 s），跨视图
  **3.37 mm**；融合 max **10.03 mm**、p95 **3.78 mm**、final **1.19 mm**
  （重放 9.9/3.6/1.4）；max 是 t=26.2 s 一次 12.4 mm 视图切换瞬态，无 67 mm
  参考切换。
- **Run 10 复跑不完整：** 停掉 motion 网关、fresh edge 后，external 267 条
  连续 60 s，但 wrist 仅 26 条（0.4–7.5 s）后 depth reader 再次 wedge；融合
  max 6.36 / p95 2.53 / final 0.27 mm，跨视图 2.77 mm（配对段）。motion 网关
  在 20:57:22 停止、腕部 ~20:57:51 停止、网关 20:58:34 才重启，不是触发者。
  结论：双视图可达成但**不可重复满 60 s**，剩余是 edge wrist-depth DDS
  reader；下一步做 per-camera ingress/participant 隔离。

证据：`evidence/hardware_20261124_tag_verify/`。

### 2.14 Split ingress（DDS reader 隔离，2026-11-24）

单进程 edge 的 reader wedge 无法在同进程重建订阅恢复，改为每相机 ingress 进程
+ fusion 进程（`split_edge.launch.py`）。ingress 只处理一个相机并发布
`TargetObservation`，fusion 订阅小观测做关联/KF；ingress 有 4 s watchdog，
退出后 launch 以 2 s 重启（新 participant）。

实机 60 s 静置：external 134 + wrist 226 条连续，跨视图 **1.55 mm**；fusion
单 track **max 2.14 / p95 1.64 / final 1.96 mm**；external ingress 在 25 s
被 watchdog 重启，fusion track 保持。部署时发现 colcon 按 mtime 跳过 core
`tracking.py`/`config.py` 重装（旧 tracker 无 `velocity_min_dt_s`，fusion 速度
爆炸到 1e8 m/s）；必须 touch 全部 core/edge 源文件并核对安装产物。core 增加
`max_track_speed_mps=0.5` / `max_target_distance_m=1.0` 安全上限。证据：
`evidence/hardware_20261124_split_edge/`。

## 3. 本轮发现并修复的问题

| # | 现象 | 根因 | 修复 |
|---|---|---|---|
| 1 | 首次运行 `ValueError: could not broadcast` | 6×6 协方差写入用了 `covariance[:3]` | 改为 `[:9]` |
| 2 | edge 单核 99%，上行仅 1 Hz | 100 Hz × 2 相机 TF 查询占满核 | 外机位 0.5 Hz、腕部 30 Hz |
| 3 | 上行/检测率约为相机一半 | 严格 `1/fps` 限速遇上 95–104 ms 抖动 | 15% 容差 |
| 4 | 编码/网络拖慢执行器 | JPEG 在 ROS 回调内 | 独立 uplink worker + depth-2 队列 |
| 5 | 运行数秒后所有回调卡数秒 | 每帧检测都发布/记录 status；rclpy 日志 `realpath` + reliable 回压 + 多线程执行器 | status 1 Hz；周期状态 best-effort；edge 单线程执行器 |
| 6 | 同一目标两条轨迹 | 两相机启动瞬态各建一条，之后各自匹配自己 | 重复轨迹合并（跨相机 6 cm/3σ，同相机严格 2 cm） |
| 7 | 真实后端烟测最初无法运行 | 主机无 torch/Ultralytics，CUDA 依赖下载限速 | 建独立 venv，安装 ONNX Runtime/OpenVINO/Ultralytics CPU 并跑通真实 `yolov8s`/OBB；后续 CUDA 版也装好 |
| 8 | 默认 D435i 配置与实机拓扑不符 | 实机是 UVC 彩色 + 仅深度 RealSense，深度未对齐彩色 | ADR 0004 提议单一 RealSense 所有者 + aligned depth；默认配置已注明 |
| 9 | `torch 2.11` 导入报 `undefined symbol: ncclDevCommCreate` | 按 2.9.1 元数据安装的 NCCL 与 2.11 不匹配 | 固定 `torch 2.9.1+cu128` / `torchvision 0.24.1+cu128`，RTX 3060 稳态 9.0 ms（FP32）/ 7.4 ms（FP16） |
| 10 | 腕部相机 `vision.launch.py` 找不到 `deptrum-ros-driver-aurora930` | third_party install 是空壳，且未 source 其工作区 | 从本地 aarch64 SDK 在独立 install 目录重建；RGB/深度恢复 |
| 11 | edge 收不到 Aurora `CameraInfo` | vendor 发布 best-effort/volatile，edge 用 reliable+transient-local | CameraInfo 订阅改为 sensor QoS；配置内参回退 |
| 12 | Fast DDS 与 vendor CycloneDDS 混用报 `ParticipantEntitiesInfo` 反序列化错误 | 跨 RMW 发现不稳定 | 实机验证统一使用 CycloneDDS；文档注明 |
| 13 | 真实腕部目标 z=+0.59 m（桌面约 -0.08 m） | 厂商 `/joint_states` 是命令缓存（joint1=+0.838、joint2–5=0），不是总线实测 | **已修正**：`wrist_tf` 用总线实测 + URDF FK + 手眼发布 `camera_optical_measured`；单姿态桌面平面 z=-0.0836 m（差 1.1 mm） |

## 4. 未验证项

- Pi 5 同时跑两路真实相机（外机位对齐 + 腕部）与腕部真实 YOLO；外机位真实 YOLO
  检测链路已通，但深度未对齐、无度量观测；
- 真实 D435i 单所有者/深度对齐（ADR 0004）；Aurora 驱动需持久化部署；
- 腕部**多姿态/固定目标**的 x/y/z 精度与运动中的 TF 滞后（单姿态桌面平面已通过）；
- 外机位的度量位姿与深度对齐；
- 关键点/pose 模型（仅验证了 detect 与 OBB）；
- 真实模型在 10 Hz 输入下的端到端实时性（GPU ROS 约 6–8 Hz，需进一步调优）；
- 实机运动集成（必须在 `jcheng_safety`/`jcheng_motion` 之下）；
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

# 实机腕部 TF（需先 source 厂商工作区，提供 ros_robot_controller_msgs）
ros2 launch jcheng_perception_edge wrist_tf.launch.py
ros2 run tf2_ros tf2_echo base_link camera_optical_measured
```

配置/代码/结果如与本文件不一致，以仓库提交和重新运行结果为准。

## 6. S1 矿石场景理解离线验证（2026-11-26）

范围：**离线、无机器人、无 DDS、无深度、无运动**。验证对象是新子模块
`jcheng_perception_scene`，不涉及现有双摄运行时。

### 6.1 命令

```bash
cd jcheng-perception
python3 scripts/ore_scene.py generate-synthetic \
  --out /tmp/ore-data --scenes 36 --empty-scenes 3 --seed 20261127
python3 scripts/ore_scene.py run \
  --dataset /tmp/ore-data --out /tmp/ore-run --config config/ore_scene.yaml
python3 scripts/ore_scene.py selftest --out /tmp/ore-selftest
python3 -m pytest tests -q
```

公开数据另用 `import-coco` 导入 Mendeley `40_70mm`（CC BY 4.0），调参
`40_0/40_1`、测试 `40_2`–`40_9`，配置
`config/ore_scene_public_40_70mm.yaml`。

### 6.2 结果

| 项 | 结果 |
|---|---|
| 测试 | **93 passed**（77 原有 + 16 新增） |
| selftest | 空场景 / 坏图 / 重叠 mask 三项通过 |
| 合成测试（39 场景 / 600 GT） | mask AP50 **0.3682**，AP50-95 **0.2114**，计数 MAE 6.51 |
| 合成延迟（640×480，CPU） | P50 14.4 ms / P95 22.3 ms |
| 公开 `40_70mm` 测试（8 图 / 9,208 GT） | AP50 **0.0429**，AP50-95 0.0122，计数 MAE 793.5 |
| 公开延迟（2592×1944，CPU） | P50 4.68 s / P95 8.53 s |
| 失败场景 | 0（合成/公开） |
| 空场景 | 3，预测 0 实例 |

### 6.3 证据与结论

- Bundle：`evidence/ore_scene_s1_20261126/`（`summary.json`、metrics、
  manifest、selftest、config、overlay、SHA-256）。
- **合成结果只证明管线/评测可工作，不证明真实矿石精度。**
- 公开结果在密集真实碎石上 AP50 约 0.043，是明确的域差距 baseline；经典
  baseline 严重欠分割，不能作为产品验收。
- 本轮**未**验证：真实矿石精度、深度/3D、多视角、GPU 后端、真机。
- 相关设计与路线：`docs/ORE_SCENE_S1.md`、`docs/ORE_SCENE_ROADMAP.md`。

### 6.4 SAM 2.1 Tiny 离线 MVE（S1.5，2026-11-26）

范围：远端 AGX Orin 离线推理，独立 `~/jcheng-s1.5` 环境；不修改
`shelf_sam3_orin`，Qwen3-VL 服务全程存活；无 DDS/设备/深度/运动。

| 项 | 结果 |
|---|---|
| 模型 | SAM 2.1 Hiera Tiny，Apache-2.0，149 MB |
| 固定 1008（8 张 held-out） | AP50 **0.508**，AP50-95 0.328，P 0.705 / R 0.591，计数 MAE 443.4，P50 43.2 s |
| 自适应 1008 + 504 回退 | AP50 **0.637**，AP50-95 0.387，P 0.617 / R 0.775，计数 MAE 335.5，P95 287.8 s |
| 504 回退救回 | `40_10` 0.141→0.567，`40_14` 0.320→0.477，`40_15` 0.089→0.549 |
| 4 图独立集（冻结 `<500` 策略） | AP50 **0.6375**，AP50-95 0.3909，P 0.649 / R 0.745，计数 MAE 246.3；`40_2` 0.245→0.546 |
| Rocks UAV 独立场景类型（2 张，负结果） | AP50 **0.301**，AP50-95 0.164，P 0.172 / R 0.605，预测 2,823 / GT 803，计数 MAE 1,010 |
| 经典 baseline（同 8 图） | AP50 0.043，AP50-95 0.012，计数 MAE 793.5 |

**注意**：回退阈值 500、tile 尺寸和合并参数是在之前的 8 张 held-out 图上
确定的；4 张独立图像（推理前选定）复现了 0.6375，但仍是同一公开数据集/
标注源，不是独立域。Rocks UAV 的 2 张 5472×3648 图则**迁移失败**：AP50
0.301、precision 0.172、预测约为 GT 的 3.5 倍，说明 1008/504 策略不能跨场景
类型直接复用。504 回退出现过分割（8 图 11,564 预测 / 9,208 GT；独立集
5,262 / 4,585），每张约 2.5–4.8 分钟。真实矿石精度仍为 **Unverified**。

- Bundle：`evidence/ore_scene_sam2_20261126/`（轻量 metrics/summary；完整
  mask 在仓库外）。
- 转换器：`python3 scripts/ore_scene.py import-sam2-tiles`（无 torch/SAM2
  导入）复现独立 4 图 1008 AP50 0.5681 与混合 AP50 0.6375 / AP50-95 0.3909；
  新增 4 个测试；含 S2/S3 后当前总计 **103 passed**。
- 设计与限制：`docs/ORE_SCENE_S1_5.md`。

### 6.5 S2 可见几何 MVE（2026-11-26）

范围：离线合成深度，只验证 mask → 可见几何的数学与失败路径；无真机/深度
采集/运行时。

| 项 | 结果 |
|---|---|
| 输入 | 480×640 深度恒 1.0 m、K fx=fy=500/cx=320/cy=240、100×100 mask、identity TF |
| 可见点 | stride 2 → 2500 点；深度有效率 1.0 |
| 可见中心 | z=1.0 m；AABB extent ≈ 0.196×0.196×0 |
| 平移 [0.1,0.2,0.3] | 中心 ≈ [0.1,0.2,1.3] |
| 零深度/空 mask/形状不一致 | `insufficient_depth` / `invalid_mask` / `ValueError` |

**注意**：合成平面只证明公式与失败路径；协方差未校准、未做 NEES，真实
D435i/Aurora 噪声与空洞未覆盖。可见中心/尺寸/OBB 不是真实质心/完整尺寸/
体积；这些字段保持 `unknown`。

- 设计与限制：`docs/ORE_SCENE_S2.md`。
- 测试：当前 103 passed（S2 含 2 个几何测试）。

### 6.6 Scene State v2 几何原型（2026-11-26）

范围：离线 schema/builder/validator；不接运行时。

- 新增 `jcheng.scene_state.v2` schema、`build_scene_v2`、`validate_scene_v2`；
- v1 记录保持不变；缺失几何写 `not_computed`，不填 0；
- 校验器拒绝 `true_centroid`/`full_extent`/`volume`/`mass` 等字段；
- 新增 `relations.py`：`touching`/`near` 两类证据边，含 contact/boundary/
  search radius/`depth_order=unknown`；不发单视角 `occludes`；
- `build_scene_v2(..., relations=...)` 校验引用/自环/重复；
  `compare_relation_sets` 提供 P/R/F1 工具，但**无独立配对标注**；
- 4 个测试；总计 **103 passed**；多视角身份仍待做；
- 设计与兼容：`docs/SCENE_REPRESENTATION.md` §4.1/§8/§9。

真实深度、NEES、独立关系标注和运行时均未验证。

### 6.7 S3 关系基准（2026-11-26）

受控合成基准：18 场景 = 6 contact（间距 0 px）+ 6 near（4 px）+ 6 clear
（80 px）；真值来自构造间距，不是关系检测器输出；评测使用 GT mask
（oracle segmentation），`touching`/`near` 折叠为 `contact`。

| 指标 | 结果 |
|---|---|
| 预测 / 真值边 | 12 / 12 |
| Precision / Recall / F1 | **1.000 / 1.000 / 1.000** |
| 场景 | 18/18 无错误 |

这是**sanity benchmark**：矩形、固定间距、GT mask；不能外推到真实矿石、
预测 mask 误差或真实关系标注。

**端到端预测 mask（负结果）**：S1 经典 baseline，39 合成场景 / 600 GT：

| 指标 | 结果 |
|---|---|
| 实例匹配率（IoU≥0.5） | **43.5%**（261/600） |
| 预测 / 真值关系边 | 20 / 490 |
| Precision / Recall / F1 | **0.850 / 0.035 / 0.067** |

结论：保留下来的关系边精度尚可，但召回被分割/匹配瓶颈压死；关系规则不是
主要瓶颈。证据：`e2e_classical_metrics.json`；设计/限制：
`docs/ORE_SCENE_S3.md`。总计 **104 passed**。

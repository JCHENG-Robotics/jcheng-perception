# JCHENG Perception

**分布式机器人视觉感知（D 方案）**：树莓派 5 负责图像采集、深度解码、TF 维护、时间戳环形缓存、3D 反投影、目标关联与卡尔曼融合；笔记本 GPU 异步承担两路 YOLO 检测。图像经压缩跨 ROS 2 传输，检测结果携带**采集时间戳**返回；Pi 5 按时间戳回溯匹配深度与 TF，两路观测按距离动态加权，输出统一目标位姿给机械臂规划。

> 状态：**v0.1.0，代码 + 离线仿真 + 单机合成端到端已验证；未接入实机。**
> 本仓库不包含模型权重，不包含厂商代码，不发布任何运动命令。

---

## 1. 为什么这样切分

| 约束 | 设计回应 |
|---|---|
| Pi 5 算力有限，YOLO 会阻塞采集/控制 | YOLO 全部在笔记本 GPU；Pi 5 只做毫秒级几何与滤波 |
| 两路相机帧率不同（Aurora 12 Hz RGB / 8 Hz 深度） | 每条消息保留采集时间戳；Pi 5 按时间戳检索深度环和 TF 环 |
| 运动中的腕相机 TF 与图像必须对齐 | 深度帧按其自身时间戳取 TF；采样环内线性 + slerp 插值 |
| 外机位与腕相机精度互补 | 按测量协方差、作用距离、置信度、遮挡做信息加权 |
| 抗遮挡 | 单视图失效时另一视图继续跟踪；轨迹短暂滑行后丢失 |
| 模型迭代不能影响控制链路 | GPU 端热加载、每相机独立模型、异步有界队列、best-effort 状态流 |
| 启动瞬态产生重复轨迹 | 跨相机重复轨迹合并（同标签、距离门限、严格同相机门限） |

## 2. 数据流

```text
        D435i (USB3, 外机位)                 Aurora 930 (腕部, USB2)
               │                                      │
               └──────────────┬───────────────────────┘
                              ▼
                 ┌──────────────────────────────┐
                 │  Pi 5 · jcheng_perception_edge│
                 │  · RGB 采集 / JPEG 压缩        │
                 │  · 深度环 + TF 环（时间戳索引）  │
                 │  · 3D 反投影 / 关联 / 加权融合   │
                 │  · 卡尔曼预测 / 30 Hz 输出       │
                 └───────┬───────────────▲───────┘
       CompressedImage   │               │  Detection2DArray
       + 采集时间戳       ▼               │  + 原采集时间戳
                 ┌──────────────────────────────┐
                 │ 笔记本 GPU · jcheng_perception_gpu│
                 │  · 每相机独立异步队列/线程       │
                 │  · YOLO（Ultralytics/OpenVINO）  │
                 │  · 关键点 / 热加载 / 延迟统计     │
                 └──────────────────────────────┘
                              │
                              ▼
                 /perception/targets  FusedTargetArray
                 → 机械臂抓取规划 / 落点复检
```

**关键契约：GPU 节点绝不改写 `header.stamp`。** 它只把 Pi 5 的采集时间原样带回。时间匹配失败时观测被丢弃，绝不用“最新帧”冒充。

## 3. 相机角色

| 相机 | 角色 | 主要职责 | 典型量程 |
|---|---|---|---|
| 车头 RealSense D435i | `external` | 粗定位、目标搜索、落点复检 | 0.15–2.0 m |
| 腕部 Aurora 930 | `wrist` | 精细抓取关键点、近距离位姿 | 0.10–0.90 m |

任务相位：`SEARCH → COARSE → ACQUIRE → FINE → DROP_RECHECK`。外机位在 `COARSE`/`DROP_RECHECK` 占主导，腕相机在 `FINE` 占主导；相位先验只是相对信息权重，绝对精度仍由距离相关的深度方差决定。

## 4. 仓库结构

```text
jcheng-perception/
├── config/
│   ├── cameras.yaml              # 真实相机/话题/噪声/量程
│   ├── cameras_synthetic.yaml    # 合成演示
│   ├── fusion.yaml               # 关联/滤波/相位先验
│   ├── fusion_demo.yaml
│   ├── frames.yaml               # 外机位静态外参回退
│   ├── gpu.yaml                  # 真实 GPU 模型路径（Ultralytics）
│   ├── gpu_mock.yaml             # 无模型演示
│   ├── gpu_onnx.example.yaml     # ONNX Runtime CPU
│   ├── gpu_openvino.example.yaml # OpenVINO IR CPU
│   └── gpu_shadow.example.yaml   # 影子模型模板
├── docs/
│   ├── ARCHITECTURE.md           # 节点职责、进程/线程、数据流
│   ├── TOPICS.md                 # 话题/QoS/字段语义
│   ├── TIMESTAMP_AND_TF.md       # 时间戳与 TF 契约
│   ├── FUSION.md                 # 加权公式、相位、滤波
│   ├── DEPLOYMENT.md             # Pi5 + 笔记本部署
│   ├── MODEL_ITERATION.md        # 模型迭代/热加载/调试
│   ├── CALIBRATION.md            # 复用/更新标定
│   ├── FAILURE_MODES.md          # 故障与降级
│   ├── EVIDENCE.md               # 本版本验证记录
│   └── adr/
├── ros2_ws/src/
│   ├── jcheng_perception_msgs/   # 临时 IDL（M2 提升到 jcheng-interfaces）
│   ├── jcheng_perception_core/   # 纯 Python，无 ROS 依赖
│   ├── jcheng_perception_edge/   # Pi5 节点 + 合成场景
│   └── jcheng_perception_gpu/    # 笔记本 GPU 检测节点
├── scripts/
│   ├── simulate_pipeline.py      # 离线全链路仿真
│   ├── run_core_tests.sh
│   └── check_repo.sh
├── tests/                        # 53 项 ROS-free 测试
└── capability.yaml
```

## 5. 快速开始

### 5.1 只跑核心（无需 ROS / 无需硬件）

```bash
python3 -m pytest tests -q
python3 scripts/simulate_pipeline.py
```

仿真包含：移动目标、外机位遮挡窗口、GPU 延迟、TF 插值、双视融合、`DROP_RECHECK`。

### 5.2 构建 ROS 2 包

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash      # 机器人；本机 Jazzy 也可构建
colcon build --packages-select \
  jcheng_perception_msgs jcheng_perception_core \
  jcheng_perception_edge jcheng_perception_gpu
source install/setup.bash
```

### 5.3 单机合成端到端演示（无模型、无 GPU）

```bash
export ROS_DOMAIN_ID=77                # 建议隔离，避免旧 DDS 参与者干扰
ros2 launch jcheng_perception_gpu synthetic_demo.launch.py
```

应看到：`/perception/targets` 中**一条**双视图轨迹，`phase=3`（FINE），权重约 `external 0.2 / wrist 0.8`，`/perception/status` 中 `match_success_rate≈1.0`。

### 5.4 真实部署

```bash
# Pi 5（机器人容器/主机）
ros2 launch jcheng_perception_edge pi5_edge.launch.py

# 笔记本 GPU
ros2 launch jcheng_perception_gpu laptop_gpu.launch.py
```

部署、DDS、相机驱动与单设备所有者冲突见 [DEPLOYMENT.md](docs/DEPLOYMENT.md)。

## 6. 输出契约

| 话题 | 类型 | QoS | 语义 |
|---|---|---|---|
| `/perception/uplink/<cam>/image_compressed` | `sensor_msgs/CompressedImage` | best effort, depth 1 | JPEG；`header.stamp` = 采集时间 |
| `/perception/detections/<cam>` | `Detection2DArray` | best effort, depth 10 | 原图坐标框/关键点；stamp 原样回传 |
| `/perception/observations/<cam>` | `TargetObservation` | best effort, depth 10 | 单相机 3D 观测（调试/证据） |
| `/perception/targets` | `FusedTargetArray` | best effort, depth 10 | 融合目标；30 Hz 预测输出 |
| `/perception/status` | `PerceptionStatus` | best effort, depth 5 | 匹配率、延迟、相位、轨迹数 |
| `/perception/gpu_status` | `GpuStatus` | best effort, depth 5 | GPU 队列/推理/模型版本 |
| `~/drop_recheck`, `~/reset` | `std_srvs/Trigger` | service | 强制落点复检 / 清空轨迹 |

完整字段见 [TOPICS.md](docs/TOPICS.md)。

## 7. 已测量的行为（合成场景，2026-11-23）

| 指标 | 实测 |
|---|---|
| 每相机上行/检测/观测 | 10 Hz / 10 Hz / 10 Hz |
| 时间匹配成功率 | 99.0–100% |
| 检测采集→边缘回调延迟 | 27–40 ms（同机、mock 检测器） |
| Edge 单回调耗时 | image 0.4 ms、depth 0.1 ms、detections 0.6 ms、publish 1.3 ms、tf 0.3 ms |
| 输出轨迹 | 1 条 confirmed，双视图融合，权重 0.21 / 0.79，phase FINE |
| 真实模型 CPU 烟测 | `yolov8s.onnx` 197.5 ms / `yolov8s.xml` 126.7 ms，各 4 个框 |
| 单元测试 | 53 passed |
| colcon 构建 | 4 包通过（Jazzy 主机） |

> 以上合成指标不是实机精度或延迟验证。真实模型烟测是**单图 CPU 推理**，不代表
> Pi 5/相机/网络的端到端性能。

> 本机已在独立 venv 安装 **ONNX Runtime 1.32.0** 与 **OpenVINO 2026.4.1**，
> 并用厂商 `yolov8s.onnx/.xml` 完成真实推理；ROS 节点也通过 venv Python 跑通了
> ONNX Runtime（233 ms/帧）。CUDA PyTorch/Ultralytics 因下载限速未安装，GPU 主机
> 可另行安装后使用 `backend: ultralytics`。

## 8. 设计边界

- **不做运动控制**：不发布 `cmd_vel`、关节、夹爪、串口/CAN 命令；输出是建议位姿。
- **不做安全判定**：规划器/安全层必须自行校验工作空间、限位与目标新鲜度。
- **不在 Pi 5 跑 YOLO**：`jcheng_perception_edge`/`core` 中没有任何检测器导入（有静态测试）。
- **不伪造深度/TF**：时间匹配失败即丢弃；腕相机没有静态外参回退。
- **模型权重不入库**：见 [MODEL_ITERATION.md](docs/MODEL_ITERATION.md) 与 [LICENSE-REVIEW.md](LICENSE-REVIEW.md)。

## 9. 参考与来源

- **机器人结构与标定**：`jcheng-manipulation` v1.0.0
  - TF 树与 `camera_optical_calibrated`：[`docs/hardware/tf_tree.md`](https://github.com/JerichoCheng/jcheng-manipulation/blob/main/docs/hardware/tf_tree.md)
  - 腕相机手眼与光学帧：[`docs/hardware/camera_frames.md`](https://github.com/JerichoCheng/jcheng-manipulation/blob/main/docs/hardware/camera_frames.md)
  - 车头 D435i 内外参（2026-11-20）：`evidence/calibration/20261120/`
- **YOLO 构建参考**：厂商 `ArmPiUltra` 的 `yolov8_detect` / `yolo_detect`
  （Ultralytics + OpenVINO，`ObjectInfo` 发布风格）。本仓库保留“每相机一个模型 +
  Ultralytics/OpenVINO 导出”的做法，但把输出改成带采集时间戳、与 ROS 解耦的
  `Detection2DArray`。
- **`lerobot_piper_local`**：本机副本只包含采集/相机补丁，**没有 YOLO 代码**；
  本仓库借鉴的是其“配置化、先体检、失败早报”的工程习惯，以及相机按序列号/键名
  统一管理的做法。YOLO 部分请以厂商 `yolov8_detect` 为准。

## 10. 文档

- [架构与线程模型](docs/ARCHITECTURE.md)
- [话题与 QoS](docs/TOPICS.md)
- [时间戳与 TF 契约](docs/TIMESTAMP_AND_TF.md)
- [融合与滤波](docs/FUSION.md)
- [部署](docs/DEPLOYMENT.md)
- [模型迭代](docs/MODEL_ITERATION.md)
- [标定](docs/CALIBRATION.md)
- [故障模式](docs/FAILURE_MODES.md)
- [验证证据](docs/EVIDENCE.md)
- ADR：[边云切分](docs/adr/0001-edge-gpu-split.md) ·
  [采集时间戳契约](docs/adr/0002-capture-timestamp-contract.md) ·
  [距离加权融合](docs/adr/0003-distance-weighted-fusion.md) ·
  [D435i 单一所有者](docs/adr/0004-camera-device-ownership.md)

## 11. 许可

许可证审查进行中；本仓库没有 `LICENSE`，默认版权保留。模型权重与第三方
推理框架的许可由使用者自行遵守，详见 [LICENSE-REVIEW.md](LICENSE-REVIEW.md)。

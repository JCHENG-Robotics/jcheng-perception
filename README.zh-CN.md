# JCHENG Perception

**简体中文** | [English](README.md)

**分布式机器人视觉感知（D 方案）**：树莓派 5 负责图像采集、深度解码、TF 维护、时间戳环形缓存、3D 反投影、目标关联与卡尔曼融合；笔记本 GPU 异步承担两路 YOLO 检测。图像经压缩跨 ROS 2 传输，检测结果携带**采集时间戳**返回；Pi 5 按时间戳回溯匹配深度与 TF，两路观测按距离动态加权，输出统一目标位姿给机械臂规划。

> 状态：**实验性** — v0.1.0（2026-11-25）：代码、离线仿真、单机合成、Pi 5 ARM64
> 合成、腕部真实相机数据链路、腕部总线实测 TF、单姿态桌面平面核验、外机位跨机
> 真实 YOLO 检测链路、ADR 0004 外机位对齐深度、split-edge 入站/融合管线，以及
> 单次 60 s 双视图 tag run 已验证；多姿态精度、可重复的完整双视图窗口与运动集成未验证。
> 本仓库不包含模型权重，不包含厂商代码，不发布任何运动命令。
> **矿石场景理解（离线 S1，2026-11-26）**：新增 ROS-free 的
> `jcheng_perception_scene` 子模块，提供实例 mask、版本化场景状态、可复现
> CLI 与 mask AP 评测。合成管线已验证；真实矿石精度**未验证**。不新增
> DDS、设备、深度或运动路径。详见 [docs/ORE_SCENE_S1.md](docs/ORE_SCENE_S1.md)。
> 本文为英文主文档的中文对照；如有歧义，以英文版为准。

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
│   ├── wrist_tf.yaml             # 腕部总线实测 TF/手眼/伺服映射
│   ├── fusion.yaml               # 关联/滤波/相位先验
│   ├── fusion_demo.yaml
│   ├── frames.yaml               # 外机位静态外参回退
│   ├── gpu.yaml                  # 真实 GPU 模型路径（Ultralytics）
│   ├── gpu_mock.yaml             # 无模型演示
│   ├── gpu_onnx.example.yaml     # ONNX Runtime CPU
│   ├── gpu_openvino.example.yaml # OpenVINO IR CPU
│   ├── gpu_tag.yaml              # AprilTag 参考点模式（定义点）
│   ├── gpu_shadow.example.yaml   # 影子模型模板
│   ├── ore_scene.yaml            # 离线矿石 S1（合成默认）
│   ├── ore_scene_public_40_70mm.yaml  # 离线矿石 S1（公开 5 MP 配置）
│   └── sam2_tiles.example.yaml   # SAM2 tile 合并 / 单图覆盖策略
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
│   ├── ORE_SCENE_RESEARCH.md     # 矿石场景调研
│   ├── ORE_SCENE_UNDERSTANDING.md# 架构与模块边界
│   ├── ORE_SCENE_ROADMAP.md      # S0-S8 路线图
│   ├── SCENE_REPRESENTATION.md   # Scene State v1 设计
│   ├── ORE_SCENE_EVALUATION.md   # 评测协议
│   ├── ORE_SCENE_S1.md           # S1 验证报告
│   ├── ORE_SCENE_S1_5.md         # SAM 2.1 离线 MVE 报告
│   ├── ORE_SCENE_S2.md           # 可见几何 MVE 报告
│   ├── ORE_SCENE_S3.md           # 关系图 MVE 报告
│   └── adr/
├── ros2_ws/src/
│   ├── jcheng_perception_msgs/   # 临时 IDL（M2 提升到 jcheng-interfaces）
│   ├── jcheng_perception_core/   # 纯 Python，无 ROS 依赖
│   ├── jcheng_perception_edge/   # Pi5 节点 + 合成场景 + wrist_tf
│   ├── jcheng_perception_gpu/    # 笔记本 GPU 检测节点
│   └── jcheng_perception_scene/  # 离线矿石 S1（ROS-free，无权重）
├── scripts/
│   ├── simulate_pipeline.py      # 离线全链路仿真
│   ├── ore_scene.py              # 离线矿石 S1 CLI
│   ├── run_core_tests.sh
│   └── check_repo.sh
├── tests/                        # 105 项 ROS-free 测试（77 原有 + 28 矿石场景/S2/S3）
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
  jcheng_perception_edge jcheng_perception_gpu \
  jcheng_perception_scene          # 仅离线矿石 S1；纯运行时构建可省略
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
# Pi 5（机器人容器/主机；先 source 厂商工作区以提供
# ros_robot_controller_msgs 和总线服务）
ros2 launch jcheng_perception_edge pi5_edge.launch.py
# 该 launch 默认同时启动 wrist_tf；也可单独启动：
# ros2 launch jcheng_perception_edge wrist_tf.launch.py

# 笔记本 GPU
ros2 launch jcheng_perception_gpu laptop_gpu.launch.py
```

部署、DDS、相机驱动与单设备所有者冲突见 [DEPLOYMENT.md](docs/DEPLOYMENT.md)。
实机腕部 TF 必须来自 `wrist_tf` 的总线实测关节，不能用厂商 `/joint_states`
命令缓存；缺失 `camera_optical_measured` 时腕部观测会被丢弃。

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
| 真实模型 CPU 烟测 | ONNX 197.5 ms / OpenVINO 126.7 ms / Ultralytics 1.04 s（首帧），各 4 个框 |
| 真实模型 GPU 烟测 | RTX 3060：Ultralytics 稳态 **9.0 ms（FP32）/ 7.4 ms（FP16）** 单图；ROS 节点平均 54.3 ms/帧 |
| 单元测试 | 77 passed（当前 `main`） |
| colcon 构建 | 4 包通过（Jazzy 主机） |

> 以上合成指标不是实机精度或延迟验证。真实模型烟测是**单图推理**，不代表
> Pi 5/相机/网络的端到端性能。

> 本机已在独立 venv 安装 **ONNX Runtime 1.32.0**、**OpenVINO 2026.4.1**、
> **PyTorch 2.9.1+cu128 / 2.6.0+cpu、torchvision 0.24.1+cu128 / 0.21.0+cpu、
> Ultralytics 8.4.216**；CUDA 版使用 RTX 3060，`torch.cuda.is_available()=True`。
> ROS 节点通过 venv Python 验证了 ONNX Runtime、Ultralytics CPU 与 CUDA。

### 7.1 Pi 5 与真实相机（数据链路 + TF + 对齐深度 + 双视图）

| 项 | 结果 |
|---|---|
| Pi 5 ARM64 构建 | 4 包通过，27.1 s |
| Pi 5 合成端到端 | 6.2/6.3 Hz、匹配 99.4%、延迟 48 ms、1 条双视图轨迹 |
| 腕部 Aurora | 驱动从本地 SDK 重建；RGB 6.7–6.8 Hz、深度 3.1–6.4 Hz、640×400 |
| 腕部真实链路（mock 蓝色检测） | GPU 4.9 Hz/0 失败；edge 输出 `blue_block-001`，FINE，wrist 权重 1.0 |
| 腕部度量 TF | `wrist_tf` 总线实测 10 Hz、1000+ 次读取 0 失败；`camera_optical_measured` |
| 桌面平面核验 | z=-0.0836 m（参考 -0.0825 m，差 1.1 mm）、倾角 1.21°、RMS 0.61 mm；**单静态姿态** |
| 跨机真实 YOLO（外机位） | 笔记本 RTX 3060 + CycloneDDS；GPU 2.06 fps / 55 ms，edge 收到 16 次检测；时间戳原样回传（运行于较早的分离相机拓扑） |
| 外机位（ADR 0004） | 2026-11-24 已执行：`armpi-ext-cam.service` 停用；`armpi-rs-depth.service` 独占 D435i 并发布彩色 + 对齐深度；配置使用 `/camera/aligned_depth_to_color/image_raw` |
| 对齐深度帧率 | 2026-11-25 重建镜像上彩色 ~6.5 Hz / 对齐深度 ~3.9 Hz（`jcheng-system` 部署验收） |
| Split edge | 每相机独立 ingress + fusion，带 respawn；静态 run 4：外机位 134 + 腕部 226 条连续观测，cross-view 1.55 mm，融合 max/p95/final 2.14 / 1.64 / 1.96 mm；外机位 ingress 看门狗在第 25 s 重启，融合轨迹存活 |
| 双视图 tag run 9 | 60 s 内 240 条外机位 + 195 条腕部观测；cross-view 3.37 mm；融合 p95 3.78 mm、final 1.19 mm、max 10.03 mm（26.2 s 处 12.4 mm 视图切换瞬态）；没有 67 mm 参考跳变 |
| 重复性 run 10 | 外机位连续 60 s（267 条观测）；腕部深度读取器在 7.5 s 卡死（26 条观测）；完整双视图窗口**尚不可重复** |
| D435i 深度恢复 | 深度缺失（`uvcvideo -71`）时重启 `armpi-rs-depth`，再 `reset-failed`/启动 ingress；见 [FAILURE_MODES.md](docs/FAILURE_MODES.md) |

> 双视图结果是单次 60 s 运行；run 10 表明腕部深度 DDS reader 仍可能卡死，完整双视图
> 窗口尚不可重复。对齐深度拓扑已部署；多姿态/固定目标精度与运动中的 TF 滞后仍未验证，
> 当前位姿不得直接用于运动。测试未发送运动命令。

## 8. 设计边界

- **不做运动控制**：不发布 `cmd_vel`、关节、夹爪、串口/CAN 命令；输出是建议位姿。
- **不做安全判定**：规划器/安全层必须自行校验工作空间、限位与目标新鲜度。
- **不在 Pi 5 跑 YOLO**：`jcheng_perception_edge`/`core` 中没有任何检测器导入（有静态测试）。
- **不伪造深度/TF**：时间匹配失败即丢弃；腕相机没有静态外参回退。
- **不信任厂商 `/joint_states` 做腕部几何**：它是命令缓存；`wrist_tf` 用总线
  实测关节 + URDF FK + 手眼发布 `camera_optical_measured`，读不到就不发布。
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
- [实机接入记录](docs/HARDWARE_BRINGUP.md)
- [验证证据](docs/EVIDENCE.md)
- [验收报告](ACCEPTANCE.md)
- ADR：[边云切分](docs/adr/0001-edge-gpu-split.md) ·
  [采集时间戳契约](docs/adr/0002-capture-timestamp-contract.md) ·
  [距离加权融合](docs/adr/0003-distance-weighted-fusion.md) ·
  [D435i 单一所有者](docs/adr/0004-camera-device-ownership.md)

## 11. 许可

许可证审查进行中；本仓库没有 `LICENSE`，默认版权保留。模型权重与第三方
推理框架的许可由使用者自行遵守，详见 [LICENSE-REVIEW.md](LICENSE-REVIEW.md)。

## 12. 矿石场景理解（离线 S1）

`jcheng_perception_scene` 是 **ROS-free、离线**子模块，提供矿石/岩石实例
分割、版本化场景状态与带 provenance 的评测。它不加入 DDS、不打开设备、不读
深度、不发布运动命令。

```bash
# 确定性合成矿石堆（640x480，36 场景 + 3 空场景）
python3 scripts/ore_scene.py generate-synthetic --out /tmp/ore-data \
  --scenes 36 --empty-scenes 3 --seed 20261127
python3 scripts/ore_scene.py run --dataset /tmp/ore-data --out /tmp/ore-run \
  --config config/ore_scene.yaml
python3 scripts/ore_scene.py selftest --out /tmp/ore-selftest

# SAM2 tile 导出 -> 场景状态（离线；不导入 torch/SAM2）
python3 scripts/ore_scene.py import-sam2-tiles \
  --dataset /tmp/ore-data --out /tmp/ore-sam2 \
  --tiles config/sam2_tiles.example.yaml --metrics
```

S1 结果（2026-11-26，仅 CPU）：

| 数据 | 范围 | mask AP50 | mask AP50-95 | 计数 MAE |
|---|---|---|---|---|
| 合成，seed 20261127 | 39 场景 / 600 GT | 0.368 | 0.211 | 6.51 |
| 公开 Mendeley `40_70mm`，测试集 | 8 图 / 9,208 GT | 0.043 | 0.012 | 793.5 |

合成结果只验证管线，不证明真实矿石精度。公开结果是一个**域差距 baseline**
（经典方法在密集真实堆上严重欠分割），不是验收结论；本轮不宣称真实矿石精度。

**S1.5（2026-11-26，离线）**：SAM 2.1 Hiera Tiny（Apache-2.0）在远端 AGX Orin
上，8 张公开 held-out 图：固定 1008 tile AP50 0.508 / AP50-95 0.328；低召回
图回退 504 tile 后 AP50 0.637 / AP50-95 0.387，但阈值/分块参数是在这 8 张图
上拟合的，且出现过分割。另用 4 张推理前选定的独立图像复现 AP50 0.6375 /
AP50-95 0.3909；仍是同一公开数据集，不是独立域。另用 2 张 Rocks UAV 无人机
图做跨场景类型检查，迁移失败（AP50 0.301、precision 0.172、约 3.5 倍过分割），
说明分块策略不具备通用性。真实矿石精度仍为 Unverified。

**S2（2026-11-26，离线合成）**：`visible_geometry` 从 mask + 米制深度 +
K + `base<-camera` 变换计算可见点数、深度有效率、可见中心、3×3 样本协方差、
AABB 与 PCA OBB；已测 1 m 合成平面、平移、零深度和空 mask。尚无真实深度、
校准协方差；可见几何不是真实质心/完整尺寸/体积。

**S3 原型（2026-11-26，离线）**：`jcheng.scene_state.v2` 通过
`build_scene_v2` 把可见几何附加到冻结的 v1 记录；缺失几何写
`not_computed`，`validate_scene_v2` 拒绝真实质心/完整尺寸/体积/质量字段。
`build_relation_graph` 增加有证据的 `touching`/`near` 边（单视角不发
`occludes`）；`compare_relation_sets` 在 18 个受控合成场景（6 contact /
6 near / 6 clear，GT mask，构造真值）上得到 P/R/F1 1.0，这是 sanity check，
不是真实关系精度。在 39 个合成场景上，经典预测 mask 匹配 43.5% 的 GT 实例，
关系 P/R/F1 为 0.850/0.035/0.067；SAM2.1 mask 匹配 76.3%，提升到
0.960/0.437/0.600；瓶颈是分割/匹配。在公开 8 图 `40_70mm` 集（9,208 个 GT
实例）上，bbox + 局部 mask 关系图可扩展到稠密场景：经典匹配 10.4%，
P/R/F1 0.787/0.015/0.030；SAM2.1 匹配 59.1%，P/R/F1 0.957/0.394/0.558。
多视角身份与运行时集成仍待做。

设计与证据：

- [S1 验证报告](docs/ORE_SCENE_S1.md)
- [S1.5 SAM 2.1 离线 MVE](docs/ORE_SCENE_S1_5.md)
- [S2 可见几何 MVE](docs/ORE_SCENE_S2.md)
- [S3 关系图 MVE](docs/ORE_SCENE_S3.md)
- [调研](docs/ORE_SCENE_RESEARCH.md) ·
  [架构](docs/ORE_SCENE_UNDERSTANDING.md) ·
  [路线图](docs/ORE_SCENE_ROADMAP.md) ·
  [场景表达](docs/SCENE_REPRESENTATION.md) ·
  [评测](docs/ORE_SCENE_EVALUATION.md)
- [证据包](evidence/ore_scene_s1_20261126/README.md)

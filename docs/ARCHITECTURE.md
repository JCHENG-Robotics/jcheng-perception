# 架构与线程模型

## 1. 拓扑

```text
robot (Pi 5, ROS 2 Humble)                         laptop GPU (ROS 2 Humble/Jazzy)
┌─────────────────────────────────────────┐        ┌─────────────────────────────┐
│ vendor camera drivers                   │        │ jcheng_perception_gpu       │
│  D435i: /camera/...                     │        │  worker[d435i_ext]  YOLO    │
│  Aurora: /depth_cam/...                 │        │  worker[aurora_wrist] YOLO  │
│           │ RGB / depth / CameraInfo    │        │       ▲          │          │
│           ▼                             │  ROS2  │       │ JPEG     │ dets     │
│ jcheng_perception_edge                  │◄──────►│       │ +stamp   │ +stamp   │
│  ingest RGB ─► JPEG worker ─────────────┼────────┼───────┘          │          │
│  ingest depth ─► depth ring             │        └──────────────────┼──────────┘
│  TF sample ─► pose ring                 │                           │
│  dets ─► time match ─► 3D ─► fuse ─► KF ┼◄──────────────────────────┘
│                 │                       │
│                 ▼ 30 Hz                │
│        /perception/targets              │
└─────────────────────────────────────────┘
```

依赖方向：`edge/gpu → core`（纯 Python），`edge/gpu → msgs`。`core` 不依赖 ROS。

## 2. 进程与线程

### 2.1 Pi 5 edge

| 线程 | 数量 | 职责 | 约束 |
|---|---|---|---|
| ROS executor | 1（`SingleThreadedExecutor`） | 图像/深度回调、检测回调、TF 采样、30 Hz 发布定时器 | 每个回调目标 < 10 ms |
| uplink worker | 每相机 1 | JPEG 编码 + `CompressedImage` 发布 | 队列深度 2，满则丢最旧 |
| DDS / tf2 | 库内部 | 传输、TF 监听 | — |

选择单线程执行器是实测结果：在 Fast DDS + Python 3.12 下，多线程执行器在回调
阻塞时会出现主线程自旋、所有回调被拖慢数秒；单线程 + I/O 工作线程稳定在
亚毫秒回调。JPEG 压缩与网络发布被移出执行器，因此慢网络/慢订阅者不会阻塞
深度摄入、TF 采样或融合定时器。

### 2.2 笔记本 GPU

| 线程 | 数量 | 职责 |
|---|---|---|
| ROS executor | 4（`MultiThreadedExecutor`） | 压缩图像回调、状态定时器 |
| detector worker | 每相机 1 | `imdecode` → YOLO → `Detection2DArray` 发布 |

回调只做 `bytes(message.data)` 复制与入队；队列满丢最旧帧，绝不反压网络或
edge。模型加载/热加载在工作线程内完成，节点启动不被模型加载阻塞。

## 3. 一次检测的完整路径

1. Pi 5 图像回调：`cv_bridge`/`imdecode` → `ingest_frame`（存 `FrameMeta`）→
   按上限频率把图像交给 uplink worker。
2. uplink worker：JPEG 编码 → `CompressedImage`，`header.stamp` = 驱动采集时间
   → best-effort 发布。
3. GPU 回调：复制字节 + 时间戳入队。
4. GPU worker：解码 → YOLO → `Detection2DArray`，`header.stamp` 原样保留，
   附 `camera_id`、`model_id`、`inference_ms`。
5. Pi 5 检测回调：转成核心 `Detection` → `ingest_detections` 暂存 → 合并窗口
   （默认 30 ms）后 `flush`。
6. `flush`：
   - 按检测时间戳找最近 `FrameMeta`（容差 50/80 ms）；
   - 找最近深度帧（容差 80/150 ms）；
   - 用**深度帧时间戳**在 TF 环中插值 `base_link ← optical`；
   - 每帧深度采样（关键点优先、框中心、网格回退）→ 3D 反投影 → 协方差；
   - 轨迹关联（标签 + 3D 门限 + 每相机一对一）→ 重复轨迹合并；
   - 双视图信息加权 → 卡尔曼更新 → 轨迹生命周期。
7. 30 Hz 定时器：把轨迹预测到当前时刻，发布 `FusedTargetArray`；1 Hz 发布
   `PerceptionStatus`。即使检测暂停，预测仍短暂输出；超时后轨迹转
   `COASTING`/`LOST`，最终不再发布。

## 4. 为什么 YOLO 不放在 Pi 5

- 推理会占用 CPU/GPU 与内存带宽，直接增加采集抖动与融合延迟。
- 模型迭代需要重启/换权重；放在笔记本可以热加载而不影响机器人节点。
- 两台机器解耦后，GPU 掉线只会让轨迹滑行/丢失，不会阻塞相机采集与 TF。
- 静态测试 `test_edge_and_core_have_no_inference_code` 防止 edge/core 意外引入
  `ultralytics`/`YOLO`。

## 5. 控制链路隔离

- 本仓库**不发布任何运动命令**，不打开串口/CAN/舵机。
- 周期状态流使用 **best-effort** QoS；慢订阅者不会通过 reliable 历史/重传
  阻塞发布者。
- 输出是“最新状态”：规划器应使用 `header.stamp` 检查新鲜度；空数组表示
  “当前无目标”，不是“保持”。
- 所有几何处理在检测回调内完成，耗时 < 1 ms；30 Hz 预测定时器独立运行。

## 6. QoS 选择

| 数据 | QoS | 原因 |
|---|---|---|
| 相机 RGB/深度 | best effort, depth 2 | 传感器流；丢帧优于积压 |
| 上行 JPEG | best effort, depth 1 | 只保留最新帧；避免旧帧排队 |
| 检测 | best effort, depth 10 | 陈旧检测无价值；防止 GPU 被慢 edge 阻塞 |
| 观测/融合目标/状态 | best effort, depth 10/5 | 周期状态；最新值语义 |
| `CameraInfo` | reliable + transient local | 需要迟到订阅者也能拿到内参 |
| 服务（drop_recheck/reset） | 默认 reliable | 一次性命令 |

## 7. 时间与延迟预算（同机合成，实测）

| 阶段 | 实测/目标 |
|---|---|
| Pi 5 图像回调（解码 + 元数据） | 0.4 ms |
| JPEG worker（640×400，q80） | 1–3 ms |
| 网络/传输（同机） | < 5 ms |
| GPU mock 检测 | 1.4 ms（真实 YOLO 预计 10–40 ms） |
| 检测回调 + 融合 | 0.6 ms |
| 采集→边缘检测回调 | 27–40 ms |
| 输出周期 | 33 ms（30 Hz） |

真实模型的端到端延迟 = 上述 + 模型推理 + 网络，应在部署后重新测量；缓冲区
默认 5 s，足以覆盖 100–300 ms 的推理延迟。

## 8. 扩展

- 增加相机：在 `cameras.yaml` 增加条目、在 `gpu.yaml` 增加模型、启动文件无需改；
  `core` 的环、融合、关联按相机数通用。
- 增加检测类别：GPU 端每相机独立 `labels`/模型；融合按标签关联。
- 影子模型：`shadow_gpu.launch.py` 在同一上行上运行候选模型，发布到
  `/perception/detections_shadow/*`，edge 不消费；生产与影子不能共用检测话题。
- 相机所有者：外机位的彩色/深度必须来自同一 RealSense 所有者并开启
  `align_depth_to_color`（见 [ADR 0004](adr/0004-camera-device-ownership.md)）。
  当前实机的 UVC 彩色 + 仅深度分离模式不满足对齐要求，不能用于度量融合。
- 多目标：关联门限 + 每相机一对一；同相机重复轨迹用严格 2 cm 门限合并，
  跨相机重复用 6 cm + 3σ 门限合并。
- 域隔离：目标架构中相机只读镜像由 `jcheng_system` 的 `jcheng_telemetry`
  提供；本仓库默认直接订阅厂商相机话题，部署在域 0/42 均可，但**不发布**
  厂商命令。

## 9. 非目标

- 不替代 `jcheng_safety`/`jcheng_motion` 的安全校验。
- 不做 6D 物体模型配准；方向来自点云主方向或关键点轴，存在 ±180°/90° 歧义，
  见 [FUSION.md](FUSION.md)。
- 不做多目标数据关联的最优解（当前为门控贪心 + 合并），目标数增加时需升级。

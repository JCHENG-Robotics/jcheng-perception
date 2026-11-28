# 矿石场景理解架构与模块边界（S0）

> 状态：S0 设计已落地为仓库内 `jcheng_perception_scene` 离线子模块；S1 只做
> RGB 离线实例分割与评测。**没有** DDS、设备、运动、深度、3D 几何或真实机器人
> 连接。本文与 `SCENE_REPRESENTATION.md` 是设计基线，不代表已实现全部阶段。
>
> **平台转向（2026-11-28）**：ArmPi Ultra 已 `Frozen`。本文的架构、模块边界
> 和离线子模块边界继续有效；S2 标定、S4 双摄/腕部、S5、S6 可操作性和 S8
> 实机闭环的平台假设改按 PiPER X 固定单元重做。阶段处置见
> [ORE_SCENE_ROADMAP.md](ORE_SCENE_ROADMAP.md) §1.1。

## 1. 目标与非目标

### 1.1 目标

把现有“YOLO 检测 + 深度反投影 + 双视角融合 + Kalman”的目标级感知，逐步扩展为
**实例级、几何级、关系级、带不确定性**的场景理解，为矿石分拣提供结构化状态：

- 实例 mask 与独立实例 ID（单场景有效）。
- 可见 3D 点云、可见中心/尺寸、深度有效率（S2）。
- 邻近/接触/遮挡等有证据的关系（S3）。
- 多视角实例关联与场景融合（S4）。
- 主动感知/下一最佳视角（S5）。
- 抓取可操作性建议（S6）。
- 向 Applications / Learning / Benchmark 提供标准场景信息（S7）。

### 1.2 非目标（本轮明确不做）

- 不新建独立仓库，不修改冻结的 `jcheng-manipulation` v1.0 基线。
- 不修改现有双摄 YOLO/融合运行时，不发布厂商 topic，不加入 0/41/42 域。
- 不输出真实质心、完整体积、质量、矿物成分、品位或经济价值。
- 不把可见 mask 的中心当抓取位姿，不直接发布运动指令。
- 不把合成/公开数据结果声称为真实矿石验收结果。

## 2. 现有生态只读审查结论

| 仓库 | 可复用 | 需要适配 | 尚未实现 | 禁止修改/禁止依赖 |
|---|---|---|---|---|
| `jcheng-perception` | `core` 的时间环、深度采样、反投影、协方差、TF、融合、跟踪；`Detection`/`TargetObservation` 数据形状；数据集脚本的 provenance 思路 | 新增 `jcheng_perception_scene`；未来把 mask 接入 edge/fusion 需要新 topic | 实例 mask、instance_id、场景状态、3D 可见几何、关系、多视角实例关联 | core/edge 不得引入 ultralytics；不得把权重写入仓库；不得破坏现有 77 个测试 |
| `jcheng-interfaces` | 版本/哈希/提案流程；`vision_msgs` 风格 | 未来需要新的感知 IDL 与 schema | 任何 detection/instance/mask/scene 消息都不存在 | 本轮**不新增/不修改**任何 IDL、schema、tag 或 contract hash |
| `jcheng-system` | gateway `/v1/state`、租约、`ExecuteMotion`、证据记录 | 未来只读场景状态镜像到 gateway 的只读 telemetry | 没有 scene/instance 字段；`PERCEPTION_UNAVAILABLE/STALE` 只有错误码，无触发点 | 不得让 perception 进入运动权限；不得绕过 lease/deadman/stop latch |
| `jcheng-applications` | recipe 校验、确定性计划、fail-closed runner、幂等提交 | 未来 recipe facts 需要能引用实例/场景 | recipe 闭集，只支持 `measured.index/at_least_rad/at_most_rad` | 不得在 recipe 中硬编码矿石业务或直接执行运动 |
| `jcheng-learning` | 原生 dataset store、provenance、DQ 校验、LeRobot 导出 | 需要新的 mask/segmentation feature 与新契约 | `jcheng.dataset.v1` 没有 mask/segmentation stream；LeRobot 导出明确不含 detections/depth | 不得把数据集/权重写入 git；不得把场景状态当动作真值 |
| `jcheng-benchmark` | suite YAML、JSONL runner、阈值/报告/provenance 标签 | 需要扩展 `data_source`（当前只有 hardware/recorded/synthetic/mock，没有 public）与图像/mask 指标 | 只支持 `position_error`/`scalar`/`success_rate`；不消费图像路径 | 不得在 benchmark 内做推理或读设备；图像/mask 只能以数字指标进入 |
| `jcheng-simulation` | 域 43 隔离、`/sim/*` 契约、truth source 标签、机器人目录 | 需要深度流与实例/语义真值导出才能做 S5/S6 仿真 | 没有深度流、没有实例 mask、没有矿石资产 | 不得让仿真进入 0/41/42；不得把仿真真值当硬件证据 |
| `jcheng-mobile` / `jcheng-teleop` | 底盘 `/cmd_vel` 意图与遥操作边界参考 | 未来矿石分拣若移动需要 Nav2/底盘协同 | 与矿石场景理解无直接接口 | 不得在 scene 模块内发布 `/cmd_vel` 或遥操作命令 |

关键事实（来自只读审查）：

- `jcheng_perception_core` 是 ROS-free 的；`Detection` 只有 label/score/bbox/
  keypoints/track_hint，**没有 mask**；`TargetObservation` 有位置、协方差、
  `valid_depth_fraction`、`occlusion_ratio`，但仍是单点目标，不是实例。
- `jcheng-interfaces` 25 个 IDL 中没有感知消息；`vision_msgs/Detection2D` 只有
  header/results/bbox/id，**没有 mask 字段**。mask 必须作为独立契约设计。
- `jcheng-system` 的 `/v1/state` 是 `RobotState`（关节、夹爪、底盘、电池、
  profile hash、truth/verification），没有 scene/target 字段。
- `jcheng-applications` 的 recipe 是闭集；`jcheng-benchmark` 只接受数值指标；
  `jcheng-learning` 的数据契约没有 mask stream。这三处决定了 S7 必须先做契约
  变更，而不是“直接接进去”。

## 3. 总体架构

```text
                    离线（本模块，ros-free）
  ┌──────────────┐   ┌──────────────────────────────┐   ┌──────────────────┐
  │ 合成数据生成  │──▶│ jcheng_perception_scene       │──▶│ scenes.jsonl      │
  │ COCO 适配器   │   │  · dataset/provenance        │   │ instances.jsonl   │
  │ 公开/录制数据 │   │  · SegmentationBackend       │   │ masks/*.png       │
  └──────────────┘   │  · mask ops / geometry(S2)   │   │ overlays/*.png    │
                     │  · scene state / relations   │   │ metrics.json      │
                     │  · evaluation / CLI          │   └──────────────────┘
                     └──────────────┬───────────────┘
                                    │ advisory only（未来 S4/S7）
                                    ▼
                 ┌──────────────────────────────────────┐
                 │ jcheng-system gateway（只读/受租约）  │
                 │  /v1/state 扩展示例（提案阶段）       │
                 │  ExecuteMotion 仍是唯一运动入口       │
                 └──────────────────────────────────────┘

  现有双摄运行时（不变）：
  D435i/Aurora → edge（时间环/深度/TF/反投影/融合）→ /perception/targets
  本模块当前不订阅、不发布、不加入任何 DDS 域。
```

### 3.1 分层

| 层 | 内容 | 约束 |
|---|---|---|
| L1 契约 | `jcheng-interfaces`（本轮不改） | 任何新消息先提案、后升版、再改哈希 |
| L2 系统 | `jcheng-system`：安全状态机、租约、仲裁、`ExecuteMotion` | 场景状态只能 advisory；不得直接运动 |
| L3 能力 | `jcheng-perception`（本模块）、mobile/learning/teleop | 不发布厂商 topic、不打开设备、不加入 41 域 |
| L4 应用 | applications/studio | 只经 gateway HTTP/WS；无 DDS/设备/租约自有 |
| OPS | 健康探针/看门狗 | 只 HTTP；不加入 DDS |

本模块属于 L3，且是**离线工具 + 未来只读场景发布者**，不是运动源。

## 4. 模块边界（实际目录）

```text
jcheng-perception/
├── config/
│   ├── ore_scene.yaml                  # 合成/默认 S1 配置
│   └── ore_scene_public_40_70mm.yaml   # 公开高分辨率碎石配置
├── ros2_ws/src/jcheng_perception_scene/
│   ├── package.xml / setup.py / setup.cfg
│   ├── schemas/
│   │   ├── jcheng.scene_state.v1.schema.json
│   │   ├── jcheng.ore_dataset.v1.schema.json
│   │   └── jcheng.ore_evaluation.v1.schema.json
│   └── jcheng_perception_scene/
│       ├── types.py        # 版本化 dataclass、有效性、未知项
│       ├── config.py       # 严格配置校验（未知键拒绝）
│       ├── synthetic.py    # 确定性合成矿石数据 + GT
│       ├── dataset.py      # 内部 manifest、COCO 适配、JSONL
│       ├── maskops.py      # bbox/局部 mask IoU/边界/接触
│       ├── backends/       # SegmentationBackend + 经典基线
│       ├── scene.py        # 实例 → SceneState + 邻接关系
│       ├── metrics.py      # mask AP/PR/计数误差/分层/延迟
│       ├── pipeline.py     # 数据集 → mask/场景/JSONL
│       ├── visualize.py    # 叠加可视化
│       └── cli.py          # 7 个子命令
├── scripts/ore_scene.py    # 薄 CLI 入口
└── tests/test_scene_pipeline.py  # 16 个离线测试
```

模块职责与禁止事项：

| 模块 | 职责 | 明确禁止 |
|---|---|---|
| `types.py` | 定义 `InstanceMask`、`Relation`、`SceneState`、`Validity`、错误码 | 不出现 `true_centroid`/`volume`/`mass`/`ore_grade` 等字段；不把 score 叫 calibrated probability |
| `config.py` | 合并默认值、拒绝未知键、范围校验、配置哈希 | 不允许静默回退到默认阈值 |
| `synthetic.py` | 生成带 provenance 的合成矿石堆与逐实例 GT | 不声称合成等于真实；不复制第三方资产 |
| `dataset.py` | 内部 manifest、COCO 多边形导入、JSONL 读写 | 不支持 RLE 时不假装支持；不复制图像到仓库 |
| `backends/` | 只接收 BGR 图，返回互斥 mask | 不读数据集、不写文件、不发布消息、不碰设备 |
| `scene.py` | 组装 SceneState；只输出有证据的 2D 邻接 | 单视角不输出遮挡方向；不对 >64 实例做 O(N²) 全图关系 |
| `metrics.py` | COCO 风格 mask AP、固定操作点 P/R、计数/分层/延迟 | 无 GT 时返回 `null`，不返回 1.0；不把合成/公开混报 |
| `pipeline.py` | 数据集→mask→场景→JSONL；记录失败 | 不把坏图当空场景；不静默跳过失败 |
| `cli.py` | 生成/导入/运行/评测/校验/自检 | 不提供任何机器人连接或运动命令 |

## 5. 数据流与时间戳

S1 离线路径：

```text
dataset.json ──▶ scene image (BGR)
      │                │
      │                ├─▶ backend.segment(image, config, scene_id)
      │                │        └─▶ List[InstanceMask]（互斥 bool mask）
      │                ▼
      ├─▶ build_scene_state(...) ──▶ SceneState
      │        ├─ relations: adjacent（仅 ≤64 实例）
      │        ├─ validity: valid / empty_scene / failure_reason / unknowns
      │        └─ field_status: algorithm_estimate / unknown
      ▼
masks/<scene>.png（uint16 标签图） + scenes.jsonl + instances.jsonl
      │
      └─▶ metrics.evaluate(...) ──▶ metrics.json
```

时间戳与坐标系：

- S1 合成数据 `timestamp_ns=0`，`frame_id` 来自配置（默认
  `camera_color_optical_frame`）；公开数据没有可靠采集时间，保持 0。
- `scene_id` 是数据集内稳定 ID；`instance_id` 形如 `<scene_id>:<NNNN>`，
  **只在当前场景有效**。跨帧/跨视角身份属于 S4，不能把同一字符串当跟踪 ID。
- S2 接入深度时必须使用与图像 capture time 匹配的深度帧和 TF，沿用
  `jcheng_perception_core` 的时间环语义；不允许“取最新深度”。

## 6. 与现有双摄管线的关系

| 现有能力 | 本模块如何对待 |
|---|---|
| `Detection`（2D 框/关键点） | 可作为未来 `SegmentationBackend` 的提示（YOLO-seg/SAM prompt），S1 未接 |
| `TargetObservation`（3D 点/协方差/遮挡比） | S2 的几何字段设计参考；S1 不重写反投影 |
| `FusedTarget`（双视角融合目标） | S4 的实例级融合要复用其信息加权思想，但不能直接把目标 ID 当实例 ID |
| 时间环/TF | S2/S4 复用；S1 不依赖 |
| 现有 77 个测试 | 新增 16 个测试后 93 个全部通过；未改动原测试 |
| `check_repo.sh` | 编译、pytest、mock 检测、仿真、禁权重；新模块不引入权重 |

**本轮不修改** edge/fusion/node、`capability.yaml`、现有消息或 launch。S1 的
CLI 是独立离线入口，不参与运行时。

## 7. 契约策略

### 7.1 本轮

- 新增的是**仓库内 schema**（`jcheng_perception_scene/schemas/`），不是
  `jcheng-interfaces` 契约，不带 `contract_hash`。
- 场景记录 `schema_version = jcheng.scene_state.v1`；数据集
  `jcheng.ore_dataset.v1`；评测 `jcheng.ore_evaluation.v1`。
- `vision_msgs/Detection2D` 没有 mask 字段；不能把 mask 塞进 `id` 或 bbox。
- 任何跨仓库消息（场景状态、实例 mask 引用、关联置信度）在 S3/S4 先提交
  `jcheng-interfaces/docs/proposals/` 提案，等待批准后再升 minor、重生成
  manifest、更新 `contract_hash`。本轮**没有**提交或修改该仓库。

### 7.2 未来消息草案（仅设计，未实现）

需要的字段能力：

- `scene_id`、`schema_version`、`stamp`、`frame_id`。
- 实例：`instance_id`（场景内）、`semantic_label`、`score` +
  `confidence_kind`、mask 引用（独立 `sensor_msgs/Image` 或压缩 mask）、
  `visible_centroid`、`visible_extent`、`depth_valid_fraction`、
  `pose_covariance`（S2 后）。
- 关系：`subject_id`、`predicate`、`object_id`、`confidence`、`evidence`。
- 有效性：`valid`、`missing_fields`、`failure_reason`、`unknowns`。

语义约束：mask 与可见几何是**算法估计**；真实质心/完整尺寸/体积/质量/品位是
**unknown**；不得用 0 或空数组冒充。

## 8. 安全、层级与工程边界

1. **advisory only**：Scene State 只描述场景，不发布 `/cmd_vel`、关节、夹爪或
   `ExecuteMotion`。
2. **不连机器人**：本模块无 rclpy、无 DDS、无 socket、无串口、无相机设备；
   `test_scene_package_has_no_ros_or_device_path` 静态检查。
3. **不进运行时域**：不加入 0/41/42/43；未来若加 ROS 节点，必须走 capability
   域 42 且只读发布，另立提案。
4. **不碰冻结基线**：`jcheng-manipulation` 未修改。
5. **不造数据**：无 GT 不计算精度；空场景是合法观测；坏图/坏配置是结构化失败。
6. **不越权**：Applications 若未来消费场景，只能经 gateway；运动仍由
   `jcheng-system` 的租约/仲裁/停止锁存决定。

## 9. 部署形态

| 形态 | 本轮 | 说明 |
|---|---|---|
| 离线 CLI（笔记本/工作站） | **已实现** | `python3 scripts/ore_scene.py ...`，CPU 即可 |
| 离线批处理/评测 | **已实现** | 合成/COCO 导入/JSONL/指标 |
| ROS 2 只读节点 | 未实现（S4/S7） | 需新契约与 capability 变更 |
| Pi 5 边缘推理 | 不计划（S1） | 重分割不上 Pi；Pi 只做几何/融合 |
| 机器人闭环 | 未实现（S8） | 需逐次操作员授权与安全评审 |

## 10. 风险与开放问题

| 风险 | 影响 | 缓解 |
|---|---|---|
| 经典 baseline 在密集真实堆上严重欠分割 | 公开 AP50 仅约 0.043 | S1.5 评估 SAM 2/MobileSAM；保留经典对照 |
| 公开标注人工校核状态未知 | 不能当金标准 | 报告中标注 `annotation_method` 与未核实项 |
| 重叠多边形被扁平化 | GT 可见面积与 COCO 原义不同 | 文档化；S2 改为逐实例 mask 存储 |
| 无深度/3D | 无法给真实质心/体积/抓取位姿 | S2 前保持 unknown，禁止下游误用 |
| 5 MP 密集场景 O(N²) 关系计算 | 运行时间爆炸 | 已设 64 实例上限并标注 `not_computed_density_limit`；S3 做空间索引 |
| 许可证 | 权重 SA/NC/AGPL 可能污染交付 | 依赖审查逐项记录；默认只用无权重 baseline |
| 场景状态被误当运动许可 | 安全风险 | advisory 字段、文档、系统层 lease/stop 边界 |

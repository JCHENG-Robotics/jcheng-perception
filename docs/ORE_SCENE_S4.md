# S4 多视角实例关联设计稿

> 日期：2026-11-26（设计）；2026-11-27 更新 S4.1/S4.2 状态。
> 状态：**Design + S4.1 + S4.2 implemented（设计稿 + 离线生成器 + 匹配器）；
> S4.3–S4.6 未实现**。
>
> 本文只定义 S4 的问题、输入输出、关联证据、数据模型提案、评测协议与验收
> 门槛。**不包含运行时代码、ROS 节点、DDS 话题、设备访问或运动路径**；也
> 不修改 `jcheng-interfaces`。文中所有阈值都是建议值，必须由 Board 确认并
> 在独立数据上标定后才能写成实现参数。S4.1/S4.2 的实现与证据见
> [`ORE_SCENE_S4_1.md`](ORE_SCENE_S4_1.md)、
> [`ORE_SCENE_S4_2.md`](ORE_SCENE_S4_2.md)。

## 1. 范围

S4 的目标是：把**外机位与腕部相机在同一时刻看到的同一块矿石**关联起来，
形成跨视角一致的场景状态；关联结果必须带证据、置信度和不确定性，不能把
关联 ID 当作客观真值。

S4 的核心问题不是“再做一个检测器”，而是：

1. 两个视角看到的是同一物理实例的**不同可见表面**，可见质心不相等；
2. 矿石外观高度相似，颜色/纹理特征区分度弱；
3. 遮挡、深度空洞、外参残差会让“最近邻”产生系统性错配；
4. 稠密矿堆中相邻实例的 3D 距离可能小于跨视角同一实例的表面差异。

因此 S4 必须显式输出 `associated` / `ambiguous` / `singleton`，禁止为了
“每个实例都有 ID”而强行合并。

### 1.1 非目标

- 不做跨帧跟踪（`track_id` 的时序生命周期属于后续阶段；S4 只做同一同步
  观测对内的跨视角关联）。
- 不输出真实质心、完整尺寸、体积、质量、品位或价值。
- 不做多视角三维重建的完整网格/TSDF；只融合**可见**证据。
- 不把现有 `FusedTarget.target_id` 当实例 ID 或 `global_id` 使用。
- 不接真机运动，不发布 `/cmd_vel`、不调用 `ExecuteMotion`。
- 不在本轮提出 `jcheng-interfaces` 变更；数据模型只是仓库内设计提案。

## 2. 输入与输出

### 2.1 输入

每个视角一份 S3 `jcheng.scene_state.v2`（或等价的内部结构）：

| 输入 | 必需 | 说明 |
|---|---|---|
| `view_id` | 是 | 例如 `d435i_ext` / `d435i_wrist`，不是相机序列号 |
| `scene_id` | 是 | 单次观测 ID；跨视角不同 |
| `timestamp_ns` | 是 | 图像采集时间；用于时间门控 |
| 实例列表 | 是 | `instance_id`、mask、`visible_geometry`、`depth_valid_fraction` |
| 相机内参 | 是 | 投影/反投影 |
| 相机外参 | 是 | 统一到 `base_link`；带来源与残差 |
| 每视角偏置模型 | 可选 | 现有 `view_bias.yaml` 的链路口径；不是实例级真值 |
| 支撑面/场景上下文 | 可选 | 平面、重力方向、关系图；只作为弱证据 |

时间同步规则：只关联同一采集窗口内的两帧；超过窗口输出
`temporal_mismatch`，不允许用“最近一帧”偷偷补齐。窗口由采集配置决定，
建议不超过一个相机帧周期，并在评测中报告 0/1 帧偏移的敏感性。

### 2.2 输出

一份关联结果（建议独立文件 `jcheng.scene_association.v1`，见 §6），包含：

- `clusters`：每个 cluster 是一个跨视角候选物理实例；
- `status`：`associated` / `ambiguous` / `singleton`；
- `members`：`view_id` + 该视角的 `instance_id`；
- `confidence` 与 `confidence_kind`（未标定前必须是
  `heuristic_uncalibrated`，不得叫 probability）；
- `evidence`：每个门控/证据的实际数值；
- `unmatched_reason`：单视角实例未关联的原因枚举；
- 每视角原始 `SceneState` 的引用（路径 + sha256），不丢单视角证据。

## 3. 关联证据与门控

### 3.1 为什么不能只用可见质心距离

同一块矿石，外机位看到侧面，腕部看到顶面；两个可见质心是**不同表面点**
的统计量，不等于同一物理中心。历史静态实验中跨视角目标中心曾出现
10–14 mm 的常数偏移（疑似外参残差；另一轮 AprilTag 实验为 3.37 mm）。
把这种偏移当成“允许误差”扩大门控，会把相邻矿石也吞进来。

因此 S4 的主证据是**互相投影支持**与**表面一致性**，可见质心距离只作为
弱先验，并且必须在应用每视角偏置后使用。

### 3.2 候选生成（由粗到细）

1. **时间门控**：`|dt| <= sync_window`；否则不进入候选。
2. **视锥/视野门控**：实例的可见点是否落在另一视角视锥内；完全在外则
   标 `outside_fov`，不强行关联。
3. **3D 包围盒粗筛**：两视角的 `visible_aabb` 在 `base_link` 下按
   `association_margin_m` 扩张后是否相交；不相交直接淘汰。
4. **尺寸相容性**：用可见包围盒的最大边长/中位边长比，超过
   `size_ratio_range` 淘汰。可见尺寸随视角变化，因此只做宽门控，不做
   精确尺寸相等判断。
5. **可见质心 Mahalanobis 距离**（弱先验）：使用 `visible_centroid_base`
   与 3×3 协方差；协方差必须叠加外参残差与偏置不确定度。该分数只用于
   排序/打破平局，不能单独决定关联。

### 3.3 主证据：互相投影支持

对候选对 `(A, B)`：

1. 把 A 的可见点（mask 内像素反投影到 `base_link`）投影到 B 的相机；
2. 统计落在 B 的实例 mask 内、且投影深度与 B 的深度图在
   `depth_tolerance_m` 内一致的点比例，记为 `support_A_to_B`；
3. 反向计算 `support_B_to_A`；
4. 取对称支持 `min(support_A_to_B, support_B_to_A)` 或调和平均，作为
   主证据；低于 `min_mutual_support` 淘汰。

该证据直接处理“看到不同表面”的问题：只要 A 的可见表面在 B 中可见且
深度一致，就支持同一实例；不需要两个可见质心重合。

注意事项：

- B 的该实例被遮挡时，`support_A_to_B` 会低；此时不能用低支持直接判
  “不同实例”，应结合 B 的可见性/遮挡比例，输出 `ambiguous` 或
  `occluded`，而不是错误合并。
- 深度容差必须来自深度噪声模型（随距离增长），不能用固定毫米数掩盖
  远处误差。

### 3.4 辅助证据

| 证据 | 作用 | 限制 |
|---|---|---|
| 投影 mask IoU | 把 A 的 mask 投到 B 平面后的 2D 重叠 | 受遮挡/外参误差影响 |
| 对称 Chamfer 距离 | 两组可见点云的表面距离，按实例尺度归一化 | 只比较可见表面，不是完整形状 |
| AABB/OBB 尺寸比 | 排除明显不同尺度的候选 | 视角导致可见尺寸变化 |
| 关系图一致性 | 邻接/接触边在另一视角的对应关系是否自洽 | 关系本身有误差，只作弱证据 |
| 支撑面一致性 | 两实例是否位于同一支撑面/高度带 | 平面估计误差 |

### 3.5 代价与门控顺序

建议使用**硬门控 + 软代价**：

```text
cost(A, B) = w_support * (1 - symmetric_support)
           + w_chamfer * normalized_chamfer
           + w_size    * size_mismatch
           + w_prior   * mahalanobis_centroid
```

硬门控（任一失败即不进入匹配）：

- 时间窗；
- 视锥；
- AABB 扩张相交；
- 尺寸比；
- 互相投影支持下限。

权重与阈值全部写入配置并记录 hash；不允许在评测后按测试集调参。建议
先用消融实验确定权重，再在独立 split 上冻结。

## 4. 匹配、歧义与失败

### 4.1 匹配

- 构造候选代价矩阵，使用匈牙利算法求全局最优；
- 只保留通过硬门控的边；
- 平局按 `(view_id, instance_id)` 稳定排序，保证确定性；
- 不允许一个实例在同一视角内被匹配两次。

### 4.2 歧义判定

对每个已匹配对，计算最优与次优候选的代价差 `margin`：

- `margin >= ambiguity_margin`：`associated`；
- `0 <= margin < ambiguity_margin`：`ambiguous`，保留 top-2 候选及证据，
  **不输出唯一 `global_id`**；
- 无候选：`singleton`，写 `unmatched_reason`。

`ambiguous` 不是失败，而是正确表达不确定性；下游不得把 ambiguous 的
top-1 当确定目标。

### 4.3 `unmatched_reason` 枚举（建议）

`outside_fov`、`occluded`、`depth_invalid`、`no_candidate`、
`size_gate`、`low_mutual_support`、`temporal_mismatch`、
`density_limit`、`ambiguous_top2`。

每个 singleton 必须能解释“为什么没关联”，不能只写 `unknown`。

### 4.4 稠密场景上限

候选生成仍是 `O(N_ext × N_wrist)` 的最坏情况；稠密矿堆需要空间索引
（体素/网格）或先按支撑面/高度带分块。超过 `max_candidates` 时输出
`density_limit`，不静默截断成“无关联”。上限与分块策略在 S4.2 实现时
用基准测试确定。

## 5. 几何融合（仅可见证据）

关联成功后，融合**可见**几何：

- 位置：各视角可见点的信息加权融合；协方差取信息矩阵之和的逆；
- 尺寸：只输出 `visible_union_aabb` / `visible_union_obb`，不输出完整
  尺寸；
- 保留每个视角的原始 `visible_centroid`、协方差、深度有效率；
- 任何融合字段的 `field_status` 仍是 `algorithm_estimate`；
- `true_centroid`/`full_extent`/`volume`/`mass` 继续为 `unknown`，校验器
  拒绝这些字段。

现有 `jcheng_perception_core.view_bias` 的“每视角常数偏置 + 协方差膨胀”
可以复用其思想，但要注意：它建模的是同一目标点的链路偏置，不是
“不同可见表面”的差异；S4 不能假设加上 `b_ext` 后两个可见质心就重合。

## 6. 数据模型提案（仓库内，不改契约）

建议独立文件，而不是把关联塞进 v2 实例：

```json
{
  "schema_version": "jcheng.scene_association.v1",
  "association_id": "assoc-20261126-0001",
  "created_at": "2026-11-26T00:00:00+00:00",
  "views": [
    {
      "view_id": "d435i_ext",
      "scene_id": "scene-ext-0001",
      "scene_state_path": "scenes/ext.json",
      "scene_state_sha256": "…",
      "timestamp_ns": 0,
      "frame_id": "base_link"
    },
    {
      "view_id": "d435i_wrist",
      "scene_id": "scene-wrist-0001",
      "scene_state_path": "scenes/wrist.json",
      "scene_state_sha256": "…",
      "timestamp_ns": 0,
      "frame_id": "base_link"
    }
  ],
  "params": {
    "sync_window_ns": 0,
    "association_margin_m": 0.0,
    "depth_tolerance_m": 0.0,
    "min_mutual_support": 0.0,
    "size_ratio_range": [0.0, 0.0],
    "ambiguity_margin": 0.0,
    "max_candidates": 0,
    "config_sha256": "…"
  },
  "clusters": [
    {
      "global_id": "assoc-20261126-0001:g0001",
      "status": "associated",
      "members": [
        {"view_id": "d435i_ext", "instance_id": "scene-ext-0001:0007"},
        {"view_id": "d435i_wrist", "instance_id": "scene-wrist-0001:0003"}
      ],
      "confidence": 0.0,
      "confidence_kind": "heuristic_uncalibrated",
      "evidence": {
        "support_ext_to_wrist": 0.0,
        "support_wrist_to_ext": 0.0,
        "symmetric_support": 0.0,
        "chamfer_m": 0.0,
        "size_ratio": 0.0,
        "centroid_distance_m": 0.0,
        "mahalanobis": 0.0
      },
      "ambiguous_candidates": null,
      "unmatched_reason": null
    }
  ],
  "singletons": [
    {
      "view_id": "d435i_ext",
      "instance_id": "scene-ext-0001:0011",
      "status": "singleton",
      "unmatched_reason": "occluded"
    }
  ]
}
```

约束：

- `global_id` 只在 `association_id` 内有效，不跨采集 session 持久化；
- 每个成员必须引用真实存在的 `(view_id, instance_id)`；
- `status=ambiguous` 时必须 `global_id: null`，并用 `ambiguous_candidates`
  列出至少两个候选（各自含 `view_id`/`instance_id`/代价/证据）；不得用
  cluster 级临时 ID 伪装成唯一关联；
- `confidence_kind` 未标定前只能是 `heuristic_uncalibrated`；
- 任何数值缺失写 `null` + 原因，不写 0。

## 7. 评测协议

### 7.1 数据集

| 类型 | 来源 | 真值 | 用途 | 限制 |
|---|---|---|---|---|
| 合成双视角 | 离线 3D 椭球/多面体矿堆，已知外参，z-buffer 渲染 mask+depth | 构造级实例对应 | 主评测、消融、敏感性 | 不是真实矿石纹理/粉尘 |
| 真实静态双摄 | 外机位 + 腕部，固定臂位，静态矿堆 | 人工配对标注的可核验子集 | 真实域 sanity | 标注成本高；只能报告子集 |
| 公开数据 | 本轮未找到带双视角实例对应的公开矿石数据集 | 不可用 | — | 不得用单视角数据伪造多视角真值 |

真实采集只允许静态、只读、逐次操作员授权；不移动机械臂，不重启 vendor
容器。腕部相机的外参随臂位变化，必须记录实际关节角/末端位姿，不能把
一个臂位的外参套到另一个臂位。

### 7.2 Split 与预注册

- 合成集按场景/随机种子划分 tune / held-out；先在小规模 tune 上确定
  门控与权重，冻结配置 hash 后在 held-out 上只跑一次；
- 真实集与合成集分开报告，不能混合成一个“总 F1”；
- 任何调参后的结果必须标注“tuned on …”，不得冒充 held-out。

### 7.3 指标

**配对级**（主要）：

- `pair_precision`：预测为同一物理实例的跨视角对中，真值也相同的比例；
- `pair_recall`：真值跨视角对中被正确关联的比例；
- `pair_f1`：两者的调和平均；
- 按实例密度、遮挡比例、深度有效率、距离分档报告。

**聚类级**：

- `cluster_purity`：每个预测 cluster 中多数真实实例的占比（按成员数加权）；
- `cluster_completeness`：每个真实实例的成员被收进同一 cluster 的比例；
- `ambiguous_rate`：输出 ambiguous 的实例/候选比例；
- `singleton_rate` 及 `unmatched_reason` 分布。

**置信度**：

- Brier score / ECE（仅在置信度被标定后报告；未标定只报告原始分数分布，
  不叫概率）。

**敏感性**：

- 外参扰动：平移 5/10 mm、旋转 0.5/1.0°；
- 时间偏移：0 / 1 帧；
- 深度噪声/空洞率；
- 遮挡比例分级；
- 密度分级（实例数/平方米）。

**Baseline 与消融**：

1. 仅可见质心最近邻；
2. 质心 + 匈牙利；
3. 本文互相投影支持 + 匈牙利；
4. 去掉关系图/支撑面证据的消融；
5. 去掉尺寸门控的消融。

必须报告 baseline，否则无法判断复杂证据是否真的有用。

### 7.4 报告规则

- 合成、真实、单视角、多视角分表；
- 每个数字带数据来源、split、配置 hash、日期；
- `ambiguous` 正确输出算正确行为，不算“未关联失败”；
- 真实集只报告人工核验过的子集，并给出核验比例；
- 未跑过的门槛不得写成“已达标”；真实矿石关联精度 **Unverified**。

## 8. 测试要求（实现阶段）

设计阶段先固定测试清单，实现时逐项落地：

1. 空场景：两视角都空 → 空 clusters，不报错；
2. 单视角缺失：一侧无实例 → 全部 singleton，原因正确；
3. 同一实例：合成已知对应 → associated，证据字段齐全；
4. 相邻不同实例：不得因质心接近而合并；
5. 歧义构造：top-2 代价接近 → ambiguous，不输出唯一 ID；
6. 时间超窗 → temporal_mismatch；
7. 外参错误/视锥外 → outside_fov 或低支持，不静默关联；
8. 置换不变性：交换视角顺序，结果 cluster 成员一致（ID 命名可不同）；
9. 确定性：同一输入重复运行，字节级/结构级一致；
10. 稠密上限：超过 `max_candidates` → density_limit，不静默截断；
11. 禁止字段：融合结果出现 `true_centroid`/`volume`/`mass` → 校验失败；
12. 引用完整性：成员引用不存在的实例 → 校验失败。

## 9. 验收标准（建议，待 Board 确认）

以下是**建议初始门槛**，不是已达成结果：

| 项 | 建议门槛 | 说明 |
|---|---|---|
| 合成 easy（低密度、无遮挡） | pair F1 ≥ 0.95 | 验证主路径 |
| 合成 dense/occluded | pair F1 ≥ 0.80 | 验证困难场景 |
| 歧义识别 | ambiguous 构造上正确标记 ≥ 0.90 | 不允许强行唯一 ID |
| 强制 ID 错误 | 0 | ambiguous 不得输出确定 `global_id` |
| 确定性/置换不变性 | 100% | 结构级一致 |
| 真实静态子集 | 只报告，不设最低 F1 | 先建立 baseline |
| 敏感性 | 报告 5/10 mm、0.5/1.0°、1 帧偏移 | 不得静默退化 |
| 运行边界 | 无 DDS/设备/运动/契约改动 | CI 静态检查 |

门槛的数值可由 Board 根据 baseline 调整；调整必须发生在 held-out 评测
之前，并记录理由。

## 10. 任务拆分

| 任务 | 内容 | 产出 | 依赖 |
|---|---|---|---|
| S4.0 | 本设计稿 | `docs/ORE_SCENE_S4.md` | S3 |
| S4.1 | 离线合成双视角生成器（3D 椭球、已知外参、mask+depth、遮挡） | `synthetic_dual.py` + 数据集 + 确定性测试（**已完成 2026-11-27**） | S2 几何 |
| S4.2 | 门控 + 互相投影支持 + 匈牙利 + 歧义判定 | `association.py`（ROS-free，**已完成 2026-11-27**） | S4.1 |
| S4.3 | 可见几何融合 + `jcheng.scene_association.v1` 校验 | schema + 校验器 + 测试 | S4.2 |
| S4.4 | 评测 CLI（配对/聚类/敏感性/消融）与预注册配置 | 指标 + 证据包 | S4.2/S4.3 |
| S4.5 | 真实静态双摄采集与人工配对子集 | 只读录制 + 标注规范 | 操作员授权 |
| S4.6 | Board 评审；如通过，再提契约/运行时提案 | 提案文档 | S4.4/S4.5 |

S4.1–S4.4 可以完全离线、无硬件；S4.5 需要单独授权；S4.6 之前不得修改
`jcheng-interfaces` 或新增 DDS 话题。

截至 2026-11-27：**S4.0/S4.1/S4.2 完成**（设计稿 + 离线生成器 + 匹配器，
114 tests；8 稠密合成场景/125 对，互相投影 F1 0.971 vs 质心 0.920）；S4.3
融合、S4.4 评测加固、S4.5 真实采集、S4.6 契约提案均**未开始**。

## 11. 风险与缓解

| 风险 | 影响 | 缓解 |
|---|---|---|
| 可见表面差异导致质心偏移 | 最近邻系统性错配 | 主证据用互相投影支持，质心只作弱先验 |
| 遮挡导致单视角缺失 | 假阴性/错误 singleton | 显式 `occluded`，不把低支持当不同实例 |
| 外参残差/热漂移 | 投影支持整体下降 | 记录外参来源与残差；协方差膨胀；敏感性报告 |
| 相邻矿石距离近 | 错误合并 | 尺寸/表面/关系证据 + ambiguity margin |
| 矿石外观相似 | 颜色/特征不可靠 | 不以外观为主证据；不引入未验证的 ReID |
| 稠密场景候选爆炸 | 延迟/内存 | 空间索引/分块；density_limit 显式输出 |
| 标注成本高 | 真实集很小 | 先合成主评测；真实只报可核验子集 |
| Schema 膨胀 | 兼容性风险 | 独立 association 文件；v2 不动；契约另案 |
| 被误用为跟踪/真值 | 下游错误决策 | `global_id` 作用域限定；confidence_kind 明确未标定 |

## 12. 安全与层级边界

- S4 是**离线/只读**能力：不加入 DDS 域 0/41/42/43，不开设备，不发运动
  命令，不调用 `jcheng-system` 的 `ExecuteMotion`；
- 关联结果只是 advisory；任何未来运行时暴露都必须先走契约提案、能力
  清单变更与 Board 评审；
- 真实采集必须逐次操作员授权；不移动机械臂、不重启 vendor 容器；
- 不把关联 ID 写入 `jcheng-interfaces` 的现有消息，不把 mask/ID 塞进
  `vision_msgs/Detection2D` 的 bbox；
- 不修改冻结的 `jcheng-manipulation`。

## 13. 开放问题（Board）

1. `global_id` 的生命周期：仅单次 association，还是跨帧/跨 session 持久化？
2. 关联输出是每帧一份，还是按时间窗聚合？窗口长度谁定？
3. 视角集合固定为外机位+腕部，还是要支持 N 个相机？N>2 时匈牙利/聚类
   策略不同。
4. 置信度标定数据从哪来、由谁标注？未标定前是否允许下游按分数排序？
5. 真实双摄人工配对标注的规范、样本量与验收人是谁？
6. 何时允许把关联结果暴露到运行时/契约？需要哪些证据门槛？
7. 是否需要为 S4 单独定义 `view_id`/外参版本/臂位记录规范？

## 14. 与现有实现的关系

- `jcheng_perception_core.weighting.fuse_observations`：目标级信息加权融合，
  可复用协方差融合思想；但它假设两视角测量同一目标点，不能直接当实例
  关联器。
- `view_bias`：每视角常数偏置 + 协方差膨胀；S4 只复用其“偏置可观测、
  残差要进协方差”的原则。
- `FusedTarget.target_id`：现有目标 ID，不是实例 ID，也不是 `global_id`。
- S2 `visible_geometry`：提供可见质心/协方差/AABB/OBB，是 S4 的输入，
  但不能被解释为完整几何。
- S3 关系图：可作为弱上下文证据，不能替代跨视角几何一致性。

## 15. 下一步

1. Board 评审本设计稿，确认 §9 门槛与 §13 开放问题；
2. S4.1 合成双视角生成器、S4.2 匹配器已完成（离线、无硬件，2026-11-27）；
3. 下一步做 S4.3 融合/序列化与 S4.4 敏感性/消融加固；S4.4 证据通过后再
   决定是否进入真实静态采集；
4. 在 S4.4 证据出来之前，不写“多视角关联已可用”，也不改任何契约。

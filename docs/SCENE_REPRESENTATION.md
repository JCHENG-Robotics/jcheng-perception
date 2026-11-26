# 场景表达与接口设计（Scene State v1）

> 状态：仓库内设计 + S1 已实现。`jcheng.scene_state.v1` **不是**
> `jcheng-interfaces` 契约，不带 `contract_hash`；跨仓库消息必须先走
> `jcheng-interfaces/docs/proposals/` 提案并获批。本文只描述 S1 已实现的 v1
> 字段，以及 S2 的扩展草案（未实现）。

## 1. 设计原则

1. **可见 != 真实**。字段名必须带 `visible_` 前缀，除非它是真实测量。
   `visible_centroid_px` 是可见 mask 的像素质心，不是物体真实质心；
   `visible_extent_px` 是可见 mask 的 2D 包围尺寸，不是完整物体尺寸。
2. **直接观测 / 算法估计 / 未知** 三者分开。`field_status` 逐字段标注
   `direct_observation`、`algorithm_estimate`、`unknown`；S1 的 mask/bbox/
   confidence 全部是 `algorithm_estimate`。
3. **缺失即缺失**。没有深度的字段不填 0、不填空数组冒充测量；`validity`
   给出 `missing_fields`、`failure_reason`、`unknowns`。
4. **实例 ID 有作用域**。`instance_id` 只在当前 `scene_id` 内有效；跨帧/跨
   视角身份是 S4 的关联结果，不能复用同一 ID 冒充跟踪。
5. **关系必须有证据**。每个关系带 `evidence` 与 `confidence`；单视角不输出
   遮挡方向。
6. **版本化**。schema 文件名与记录内 `schema_version` 同时升版；v1 记录不
   被静默改写。

## 2. 顶层结构

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `schema_version` | string | 是 | 固定 `jcheng.scene_state.v1` |
| `scene_id` | string | 是 | 数据集/采集内稳定场景 ID |
| `timestamp_ns` | int ≥0 | 是 | 采集时间；S1 合成/公开数据为 0，不伪造 |
| `frame_id` | string | 是 | 观测坐标系；S1 默认 `camera_color_optical_frame` |
| `image` | object | 是 | `path`、`sha256`、`width`、`height`；坏图为 null |
| `observation_source` | object | 是 | `kind`（synthetic/public/recorded/unlabelled_real）、`dataset_id`、`annotation_method`、`license`、`dataset_manifest_sha256` |
| `backend` | object | 是 | `id`、`version`、`family`、`calibrated`、`description` |
| `instances` | array | 是 | 实例列表；空数组合法（空场景） |
| `relations` | array | 是 | 关系列表；空数组合法 |
| `field_status` | object | 是 | 字段→`direct_observation`/`algorithm_estimate`/`unknown`/`not_computed_in_s1`/`not_computed_density_limit` |
| `validity` | object | 是 | 有效性、缺失、失败原因、未知项 |
| `errors` | array | 是 | 结构化错误（code/message/stage/recoverable/context） |
| `timing` | object | 是 | `segmentation_ms`、`total_ms` 等毫秒 |

### 2.1 `validity`

| 字段 | 类型 | 说明 |
|---|---|---|
| `valid` | bool | 场景是否可用于下游；坏图/后端异常为 false |
| `empty_scene` | bool | 后端确实没有观测到实例；这是合法观测，不是错误 |
| `missing_fields` | string[] | 缺失字段名（如 `depth`、`geometry`） |
| `failure_reason` | string/null | 失败码；失败时不得为 null |
| `unknowns` | string[] | 当前无法观测的量：`occlusion_direction`、`true_centroid`、`full_extent`、`volume`、`mass`、`material_composition`、`ore_grade`、`economic_value` |

## 3. 实例字段

| 字段 | 类型 | 说明 | S1 状态 |
|---|---|---|---|
| `instance_id` | string | `<scene_id>:<NNNN>`，场景内唯一 | 已实现 |
| `semantic_label` | string | S1 固定 `ore`；未来可扩展岩石/杂质类别 | 已实现 |
| `detection_confidence` | float [0,1] | 后端分数；**不是校准概率** | 已实现 |
| `confidence_kind` | string | S1 为 `heuristic_uncalibrated` | 已实现 |
| `bbox_xyxy` | int[4] | 可见 mask 的包含式像素框 | 已实现 |
| `mask_path` | string/null | 同一 label image 的相对路径 | 已实现 |
| `mask_sha256` | string/null | label image 哈希 | 已实现 |
| `mask_index` | int ≥1 | 在 label image 中的像素值 | 已实现 |
| `mask_area_px` | int ≥1 | 可见 mask 像素数 | 已实现 |
| `visible_centroid_px` | float[2] | 可见 mask 像素质心；不是真实质心 | 已实现 |
| `visible_extent_px` | int[2] | 可见 mask 的宽高；不是完整尺寸 | 已实现 |
| `geometry` | object | S1 为 `{status: not_computed_in_s1, reason: rgb_only_pipeline}` | 已实现（占位） |
| `attributes` | object | 后端内部证据（面积、距离、圆度等） | 已实现 |

**禁止字段**（v1 schema 与 CLI 校验都会拒绝）：

- `true_centroid` / `true_center`
- `full_extent`
- `volume`
- `mass`
- `material_composition`
- `ore_grade`
- `economic_value`

理由：单视角 RGB 没有可靠证据支持这些量；写 0 或估计值都会被下游误用。

## 4. 关系字段（S1 仅 2D 邻接）

| 字段 | 类型 | 说明 |
|---|---|---|
| `subject_id` | string | 实例 ID |
| `predicate` | enum | S1 只有 `adjacent` |
| `object_id` | string | 另一个实例 ID |
| `confidence` | float [0,1] | 接触像素/边界像素的启发式比例 |
| `evidence` | object | `contact_px`、`subject_boundary_px`、`object_boundary_px`、`search_radius_px`、`mask_iou`、`depth_order`、`note` |
| `source` | string | `2d_mask` |

语义：

- `adjacent` 是**无向**的 2D 接触证据；不表示物理连接、支撑或遮挡。
- `mask_iou` 对互斥 mask 恒为 0；保留它是为了证明“没有像素重叠”。
- `depth_order` 固定 `unknown`；单视角 RGB 不足以判断谁遮挡谁。
- 实例数 > `adjacency_max_instances`（当前 64）时，不计算关系，
  `field_status.adjacency = not_computed_density_limit`。这是性能边界，不是
  “没有关系”的证据。

## 5. 不确定性表达

| 量 | S1 | S2/S4 设计 |
|---|---|---|
| mask 置信度 | 启发式 score，`confidence_kind=heuristic_uncalibrated` | 校准（温度/等渗）需独立验证；未校准前不得叫 probability |
| 可见几何 | 无 | `visible_centroid_base` + 3×3 协方差；深度有效率 |
| 深度 | 无 | `depth_valid_fraction`、无效点比例、深度噪声模型 |
| 跨视角关联 | 无 | `association_confidence` + 证据（距离/尺寸/特征） |
| 实例身份 | 场景内 ID | S4 跨视角关联；不得把关联 ID 当真值 |
| 未知 | `unknowns` 列表 | 继续保留；新增字段必须给 `field_status` |

**报告规则**：任何下游展示必须同时显示数据来源与 `validity`；不能只取
`detection_confidence` 排序后当“确定目标”。

## 6. ID 作用域与生命周期

```text
scene_id = "syn-20261127-004"          # 一次观测
instance_id = "syn-20261127-004:0007"  # 仅在该 scene 内有效
mask_index = 7                         # 仅在该 label image 内有效
```

- 不同场景可以出现相同的 `mask_index`；不能跨场景合并。
- 同一物理矿石在下一帧/另一视角会得到新的 `instance_id`；S4 才建立
  `track_id`/`global_id`，并必须带关联置信度。
- `target_id`（现有 `FusedTarget`）与 `instance_id` 不是同一概念；不得直接
  互换。

## 7. 示例（v1，节选）

```json
{
  "schema_version": "jcheng.scene_state.v1",
  "scene_id": "syn-20261127-004",
  "timestamp_ns": 0,
  "frame_id": "camera_color_optical_frame",
  "image": {
    "path": "/data/synthetic/images/syn-20261127-004.png",
    "sha256": "…",
    "width": 640,
    "height": 480
  },
  "observation_source": {
    "kind": "synthetic",
    "dataset_id": "synthetic_20261127",
    "annotation_method": "procedural_ground_truth",
    "license": "CC0-1.0",
    "dataset_manifest_sha256": "…"
  },
  "backend": {
    "id": "classical_watershed",
    "version": "1.0.0",
    "family": "classical_cv",
    "calibrated": false,
    "description": "Otsu foreground, distance markers, watershed; heuristic score"
  },
  "instances": [
    {
      "instance_id": "syn-20261127-004:0001",
      "semantic_label": "ore",
      "detection_confidence": 0.78,
      "confidence_kind": "heuristic_uncalibrated",
      "bbox_xyxy": [102, 88, 171, 160],
      "mask_path": "masks/syn-20261127-004.png",
      "mask_sha256": "…",
      "mask_index": 1,
      "mask_area_px": 4380,
      "visible_centroid_px": [136.2, 124.7],
      "visible_extent_px": [70, 73],
      "geometry": {
        "status": "not_computed_in_s1",
        "reason": "rgb_only_pipeline"
      },
      "attributes": {"area_px": 4380, "circularity": 0.71}
    }
  ],
  "relations": [
    {
      "subject_id": "syn-20261127-004:0001",
      "predicate": "adjacent",
      "object_id": "syn-20261127-004:0002",
      "confidence": 0.31,
      "evidence": {
        "contact_px": 42,
        "subject_boundary_px": 136,
        "object_boundary_px": 121,
        "search_radius_px": 5,
        "mask_iou": 0.0,
        "depth_order": "unknown",
        "note": "undirected 2-D adjacency; no occlusion direction"
      },
      "source": "2d_mask"
    }
  ],
  "field_status": {
    "mask": "algorithm_estimate",
    "bbox_xyxy": "algorithm_estimate",
    "detection_confidence": "algorithm_estimate",
    "visible_centroid_px": "algorithm_estimate",
    "visible_extent_px": "algorithm_estimate",
    "adjacency": "algorithm_estimate",
    "occlusion_direction": "unknown",
    "true_centroid": "unknown",
    "full_extent": "unknown",
    "volume": "unknown",
    "mass": "unknown",
    "material_composition": "unknown",
    "ore_grade": "unknown",
    "economic_value": "unknown"
  },
  "validity": {
    "valid": true,
    "empty_scene": false,
    "missing_fields": [],
    "failure_reason": null,
    "unknowns": [
      "occlusion_direction", "true_centroid", "full_extent", "volume",
      "mass", "material_composition", "ore_grade", "economic_value"
    ]
  },
  "errors": [],
  "timing": {"segmentation_ms": 14.2, "total_ms": 16.8}
}
```

## 8. S2 几何扩展草案（未实现）

在 `instances[].geometry` 内增加：

| 字段 | 类型 | 语义 |
|---|---|---|
| `status` | enum | `computed`/`insufficient_depth`/`tf_unavailable`/`not_computed_in_s2` |
| `visible_points_uri` | string/null | 可见点云文件引用（不内嵌大数组） |
| `visible_point_count` | int | 有效可见点数 |
| `visible_centroid_base` | float[3] | 可见点云质心，**不是真实质心** |
| `visible_centroid_covariance` | float[9] | 3×3 协方差 |
| `visible_aabb` | object | `center`/`extent`/`frame_id` |
| `visible_obb` | object | `center`/`quaternion`/`extent`/`frame_id` |
| `depth_valid_fraction` | float [0,1] | mask ROI 内有效深度比例 |
| `depth_frame_id` | string | 深度坐标系 |
| `support_plane` | object/null | 支撑面法向/偏移/内点率（若可估） |

约束：

- 所有 `visible_*` 字段只描述可见部分。
- `full_extent`/`volume`/`mass` 仍为 `unknown`，除非 S4 多视角 + 先验 + 明确
  证据；即使估计，也必须 `field_status=algorithm_estimate` 并给区间。
- 深度点必须来自与图像 capture time 匹配的帧；不得取最新帧。

## 9. 版本与兼容

| 版本 | 状态 | 兼容策略 |
|---|---|---|
| `jcheng.scene_state.v1` | S1 已实现，待 Board 评审 | 新增可选字段可升 minor；删除/改语义升 major；读取方必须检查 `schema_version` |
| `jcheng.ore_dataset.v1` | S1 已实现 | 同上 |
| `jcheng.ore_evaluation.v1` | S1 已实现 | 同上 |
| `jcheng.scene_state.v2` | S2 设计草案 | 不破坏 v1 读取；迁移工具在 S3 提供 |

跨仓库契约：

- 本轮**不**修改 `jcheng-interfaces`。
- S3/S4 需要新消息时，先提交提案，内容至少包括：字段、语义、未知项、
  兼容性、安全边界、测试计划、许可证/依赖。
- `vision_msgs/Detection2D` 无 mask 字段；mask 不得塞进 bbox/id。建议独立
  mask 消息或 `sensor_msgs/Image` + 显式 `instance_id` 映射。

## 10. 校验规则（代码与测试）

1. `schema_version` 必须精确匹配。
2. `scene_id` 非空；`image.width/height > 0`。
3. `instance_id` 场景内唯一；`mask_index ≥1`；mask 与图像同尺寸。
4. 实例 mask 必须互斥（partition）；重叠是后端 bug，抛
   `MASK_OVERLAP`，不是关系。
5. `detection_confidence ∈ [0,1]`；`confidence_kind` 必须说明是否校准。
6. 关系两端必须引用存在的实例；`predicate` 在白名单内。
7. `valid=false` 必须有 `failure_reason`。
8. 实例对象不得出现禁止字段（volume/mass/grade 等）。
9. `unknowns` 必须是已知的未知项集合；不得把“没算”写成“0”。

## 11. 开放问题（待 Board）

1. `semantic_label` 是否在 S3 扩展为多类（矿石/废石/杂质/粉尘）？类别体系
   由谁维护？
2. 实例 mask 的存储格式：独立 PNG label image 已够用；跨仓库消息是否传
   RLE/压缩 mask？
3. `scene_id` 的生成规则：采集 session + 帧号，还是内容哈希？跨仓库一致性。
4. S4 的 `global_id` 生命周期与失效策略；是否需要持久化。
5. 是否允许 S2 输出“估计体积/质量”的区间值？本轮建议继续禁止，直到有
   多视角 + 密度先验 + 独立验证。

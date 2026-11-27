# S4.3 可见几何融合与关联记录校验

> 日期：2026-11-27。状态：**Partially verified（仅合成稠密场景，离线）**。
>
> 本轮在 S4.2 的 `associated` cluster 上融合**可见**几何，并实现
> `jcheng.scene_association.v1` 的结构/引用/禁止字段校验与 JSONL 序列化。
> 不接真实双摄、不做运行时暴露、不修改 `jcheng-interfaces`。

## 1. 实现

`jcheng_perception_scene/association.py` 新增：

- `fuse_association(scene, record)`：只对 `associated` cluster 附加
  `fused_visible_geometry`；`ambiguous`/`singleton` 不融合；
- `validate_association_record(record, scene=None, require_fused=False)`：
  - schema 版本、scene/view、成员数量和视角覆盖；
  - `associated` 必须有唯一 `global_id`；`ambiguous` 必须为 `null` 且至少
    两个候选；
  - 成员引用必须存在于对应视角实例（传入 `scene` 时）；
  - 融合几何必须为 `base_link`、`algorithm_estimate`，协方差对称半正定、
    AABB 合法、融合质心在 AABB 内、保留两份 `member_geometry`；
  - 递归拒绝 `true_centroid`/`true_center`/`full_extent`/`volume`/`mass`/
    `ore_grade`/`economic_value` 等字段；
- `write_association_jsonl` / `read_association_jsonl`：UTF-8 JSONL，
  可选逐行结构校验。

CLI：

```bash
python3 scripts/ore_scene.py fuse-association \
  --dataset <dual-view-dataset> \
  --out <fused.jsonl> \
  [--strategy mutual_projection|centroid]
```

每个关联场景输出一份 `jcheng.scene_association.v1` 记录，`associated`
cluster 带 `fused_visible_geometry`。

## 2. 融合方法

对 cluster 的两个成员（`ext`/`wrist`）：

- 输入：各自可见点、可见质心 `c_i`、3×3 协方差 `Σ_i`；
- 信息融合：
  `Λ = Σ Σ_i^{-1}`，`Σ_f = Λ^{-1}`，`c_f = Σ_f Σ Σ_i^{-1} c_i`；
- 可见 AABB：两个可见点并集的 min/max；
- 可见 OBB：并集点 PCA（特征值降序、右手系），中心 + 四元数 + 三轴
  extent；
- 保留 `member_geometry`（每视角的可见质心、点数、协方差、遮挡比例）。

输出字段：

```json
{
  "fused_visible_geometry": {
    "frame_id": "base_link",
    "method": "information_weighted_visible_centroids",
    "fused_visible_centroid_base_m": [0, 0, 0],
    "covariance_m2": [[...]],
    "visible_aabb": {"min_base_m": [...], "max_base_m": [...]},
    "visible_obb": {"center_base_m": [...], "quaternion_xyzw": [...],
                     "extent_m": [...]},
    "visible_points": 600,
    "member_geometry": [{"view_id": "ext", "...": "..."}, {"view_id": "wrist"}],
    "field_status": "algorithm_estimate",
    "unknowns": ["true_centroid", "full_extent", "volume", "mass",
                 "ore_grade", "economic_value"],
    "note": "visible surfaces only; not a complete object or true centroid"
  }
}
```

**命名约束**：字段叫 `fused_visible_centroid`，不是 `true_centroid`；它只是
两个可见表面统计量的融合，不能当物理中心。`unknowns` 明确保留真实质心、
完整尺寸、体积、质量、品位、价值为未知。

## 3. 证据

沿用 S4.2 的 8 个稠密合成场景 + 1 个空场景（125 条构造对，S4.2 匹配结果）：

| 项 | 值 |
|---|---|
| associated / ambiguous cluster | 118 / 0 |
| 已融合 cluster | **118/118** |
| singleton | 23 |
| 可见点/ cluster | min 76 / median 600 / max 600 |
| 可见 AABB 对角线 | 0.021–0.152 m，中位数 0.091 m |
| 融合协方差 trace | 1.67e-5–4.76e-4 m² |
| 校验 | 结构/引用/PSD/AABB/禁止字段全部通过 |
| JSONL round-trip | 完全相等 |

证据：`evidence/ore_scene_s4_3_20261127/`（`fused_associations.jsonl`、
`summary.json`、README）。

## 4. 测试

新增 3 个测试（S4.3 结束时总计 **117 passed**）：

1. 融合记录通过校验；associated 有几何、ambiguous 为 null；注入
   `true_centroid` 被拒绝；
2. JSONL 写入/读取完全相等，逐行校验通过；
3. `fuse-association` CLI 生成可校验的融合记录。

## 5. 限制

1. 融合的是**可见表面**；不是真实质心、完整尺寸、体积、质量。
2. 椭球代理、固定合成外参、理想深度；无真实噪声/标定误差。
3. 继承 S4.2 的匹配结果：7 条漏关联不会被融合修复；错误合并会传播。
4. `ambiguous` 在本合成集为 0；歧义路径由 S4.2 单测覆盖，不代表真实
   歧义率。
5. 真实矿石几何/关联精度 **Unverified**；无运行时/契约改动。

## 6. 下一步

S4.4：外参扰动、时间偏移、遮挡分层的敏感性/消融与预注册配置；在 S4.4
证据通过前，不接真实双摄、不暴露运行时。

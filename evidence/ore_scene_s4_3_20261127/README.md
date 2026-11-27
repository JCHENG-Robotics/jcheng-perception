# S4.3 可见几何融合与关联校验证据（2026-11-27）

状态：**Partially verified（仅合成稠密场景，离线）**。融合结果只描述**可见
表面**；不是真实质心、完整尺寸、体积或质量。没有真实矿石/硬件/运行时验证。

## 1. 目的

在 S4.2 的 `associated` cluster 上：

- 融合两视角的可见点/可见质心/协方差；
- 生成可见 AABB 与 PCA OBB；
- 保留两个单视角成员几何；
- 用 `jcheng.scene_association.v1` 校验器检查引用、歧义、禁止字段与
  协方差合法性；
- 验证 JSONL 序列化/反序列化一致。

`ambiguous` 不融合；`singleton` 不融合；两者都保留原始证据。

## 2. 方法

- 对每个成员：可见质心 `c_i`、协方差 `Σ_i`；
- 信息矩阵 `Λ_i = Σ_i^{-1}`，融合：
  `Λ = Σ Λ_i`，`Σ_f = Λ^{-1}`，`c_f = Σ_f Σ Λ_i c_i`；
- 可见 AABB：两视角可见点并集的 min/max；
- 可见 OBB：并集点 PCA（特征值降序、右手系），中心/四元数/三轴 extent；
- 输出字段名使用 `fused_visible_centroid_base_m`，不写 `true_centroid`；
- `unknowns` 明确列出 true centroid/full extent/volume/mass/ore grade/
  economic value；
- 校验器拒绝融合几何中的 `true_centroid`、`full_extent`、`volume`、
  `mass`、`ore_grade` 等字段，要求协方差对称半正定、AABB 合法、成员
  几何为 2 份、融合质心在 AABB 内。

## 3. 结果

数据集沿用 S4.2 的 8 个稠密合成场景 + 1 个空场景（`scene_radius=0.10`，
16–24 实例/场景，125 条构造对）：

| 项 | 值 |
|---|---|
| 关联 cluster | 118 associated / 0 ambiguous |
| 已融合 cluster | **118/118** |
| singleton | 23 |
| 可见点/融合 cluster | min 76 / median 600 / max 600 |
| 可见 AABB 对角线 | 0.021–0.152 m，中位数 0.091 m |
| 融合协方差 trace | 1.67e-5–4.76e-4 m² |
| 结构/场景引用校验 | 全部通过 |
| 协方差对称/PSD、AABB、禁止字段 | 全部通过 |
| JSONL round-trip | 完全相等 |
| 测试 | **117 passed**（新增 3 个 S4.3 测试） |

注意：这里的 118 个 cluster 继承 S4.2 的匹配结果；S4.2 的 7 条漏关联不会
被融合修复。融合只保证“给定 cluster 后，可见几何与结构合法”。

## 4. 文件

- `fused_associations.jsonl`：9 个场景的关联 + 融合记录；
- `summary.json`：计数、几何统计、检查、限制；
- `SHA256SUMS`：本目录校验和；
- 复现命令：

```bash
python3 scripts/ore_scene.py generate-dual-view \
  --out /tmp/ore_s4_3 --scenes 8 --empty-scenes 1 --seed 20261127 \
  --width 480 --height 360 --min-instances 16 --max-instances 24 \
  --scene-radius 0.10
python3 scripts/ore_scene.py fuse-association \
  --dataset /tmp/ore_s4_3 --out /tmp/ore_s4_3/fused.jsonl
```

## 5. 限制

1. 融合是**可见表面**的统计，不是完整物体；不可解释为真实质心/体积。
2. 椭球代理、固定合成外参、理想深度；无真实噪声/标定误差。
3. 继承 S4.2 的匹配错误（漏关联、可能的错误合并会传播）。
4. 真实矿石关联/几何精度 **Unverified**。
5. 无 DDS/设备/运动/`jcheng-interfaces` 改动；不是跨仓库契约。

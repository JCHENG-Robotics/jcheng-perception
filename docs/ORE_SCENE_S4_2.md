# S4.2 双视角关联匹配器

> 日期：2026-11-27。状态：**Partially verified（仅合成稠密场景，离线）**。
>
> 本轮在 S4.1 生成器上实现候选门控、互相投影支持、匈牙利匹配和
> `ambiguous` 判定，并报告与“仅可见质心”baseline 的对比。**没有真实矿石、
> 真实标定误差或运行时验证**；所有数字来自合成椭球场景。

## 1. 实现

`jcheng_perception_scene/association.py`（ROS-free，只依赖 numpy/OpenCV）：

- 读取 S4.1 数据集，把每视角实例的 label 像素 + metric depth 反投影成
  `base_link` 可见点；
- 对每个候选对计算：
  - 可见 AABB 相交（膨胀 `association_margin_m`）；
  - 尺寸比；
  - **互相投影支持**：把 A 的可见点投到 B，若 A 点在 B 表面之前/之上且
    B 深度有效，则计入分母；落在候选实例 mask 内且深度一致才计为匹配；
  - 对称 Chamfer 距离；
  - 可见质心 Mahalanobis 距离（弱先验）。
- 硬门控：时间/场景同步由输入保证；AABB、尺寸、支持像素数、互相支持、
  Chamfer；未通过门控的候选不进入匹配。
- 代价：
  `w_support*(1-support) + w_chamfer*chamfer_ratio + w_size*(size-1) + w_prior*mahalanobis_ratio`
- 匈牙利匹配（纯 Python，无 scipy 依赖）；对每个已匹配对计算 top-2 代价
  margin，低于 `ambiguity_margin` 输出 `ambiguous`，**不输出唯一
  `global_id`**。
- 未匹配实例输出 `singleton` + 原因：`no_candidate`、`size_gate`、
  `low_mutual_support`、`insufficient_visible`、`matched_elsewhere`、
  `ambiguous_elsewhere`、`not_selected` 等。

输出 schema：`jcheng.scene_association.v1`（仓库内，不是跨仓库契约）。

## 2. 评价协议

- 真值：S4.1 `pairs.jsonl` 的构造对；
- 只把 `associated` 的两成员 cluster 计为预测对；
- `ambiguous` 不强行计为正确/错误对，单独统计；
- 指标：pair precision/recall/F1、ambiguous 数、forced-ID 错误、singleton
  原因分布、按密度分档、每场景延迟；
- baseline：`strategy=centroid`，同样 AABB/尺寸门控 + 匈牙利，但代价只用
  可见质心 Mahalanobis 和尺寸，不使用互相投影支持。

## 3. 结果

数据集：`scene_radius=0.10 m`、每场景 16–24 个实例、8 个稠密场景 + 1 个
空场景，共 165 个实例、**125 条真值对**。固定 seed 20261127，未在测试集
上调参。

| 策略 | 预测对 | 正确 | Precision | Recall | F1 | ambiguous | forced-ID |
|---|---|---|---|---|---|---|---|
| **互相投影 + 匈牙利** | 118 | 118 | **1.000** | **0.944** | **0.971** | 0 | 0 |
| 仅可见质心 + 匈牙利 | 112 | 109 | 0.973 | 0.872 | 0.920 | 10 | 0 |

- 互相投影策略没有错误合并（precision 1.0），但漏了 7 条真值对：
  22 个 singleton 报 `matched_elsewhere`（竞争/遮挡导致未入选），1 个
  `no_candidate`（AABB/可见点门控失败）。
- 质心 baseline 有 10 个 ambiguous、3 个错误合并；在稠密场景中，
  可见质心不是可靠主证据。
- 延迟（Python、单进程、480×360、16–24 实例）：互相投影 p50 约 0.38 s /
  p95 约 0.42 s；质心 baseline p50 约 0.36 s / p95 约 0.41 s。
  **不是实时结论**。
- 空场景：0 clusters、0 singleton、0 错误。

## 4. 复现

```bash
python3 scripts/ore_scene.py generate-dual-view \
  --out /tmp/ore_s4_2 --scenes 8 --empty-scenes 1 --seed 20261127 \
  --width 480 --height 360 --min-instances 16 --max-instances 24 \
  --scene-radius 0.10

python3 scripts/ore_scene.py evaluate-association \
  --dataset /tmp/ore_s4_2 --out /tmp/ore_s4_2/mutual/metrics.json

python3 scripts/ore_scene.py evaluate-association \
  --dataset /tmp/ore_s4_2 --out /tmp/ore_s4_2/centroid/metrics.json \
  --strategy centroid
```

证据：`evidence/ore_scene_s4_2_20261127/`（`mutual_metrics.json`、
`centroid_metrics.json`、`summary.json`、本目录 README）。

## 5. 测试

新增 4 个测试（S4.2 结束时总计 114；S4.3 后当前 **117 passed**）：

1. 合成稠密数据上恢复构造对，检查 P/R/F1 下限、无 forced-ID；
2. 对称双实例构造 → 两个 cluster 都是 `ambiguous`，`global_id=null`；
3. 视角顺序置换结果一致；空场景无 cluster/singleton；
4. CLI `evaluate-association` 生成 metrics + associations。

## 6. 限制

1. **椭球代理**；没有真实矿石几何、纹理、粉尘、水膜或破碎面。
2. 固定合成外参，没有真实标定残差、热漂移、臂部遮挡或同步误差；
   设计稿中的 5/10 mm、0.5/1.0° 敏感性尚未实现。
3. 稠密集合仍是合成场景；真实矿石关联精度 **Unverified**。
4. 召回 0.944，不是“全部关联”；`matched_elsewhere` 与 `no_candidate`
   需要后续诊断（候选门控、遮挡可见性、Hungarian 竞争）。
5. `ambiguous` 在本合成集为 0；只证明代码路径（单元测试）可用，不证明
   真实歧义识别率。
6. 没有运行时/DDS/设备/运动/接口契约改动；结果不能用于机器人决策。

## 7. 下一步

1. S4.3：可见几何融合 + `jcheng.scene_association.v1` 校验/序列化（已完成，
   见 [`ORE_SCENE_S4_3.md`](ORE_SCENE_S4_3.md)）；
2. S4.4：评测 CLI 的敏感性/消融（外参扰动、时间偏移、遮挡分层）与
   预注册配置；
3. 在 S4.4 证据通过前，不接真实双摄、不暴露运行时。

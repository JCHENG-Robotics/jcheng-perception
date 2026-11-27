# S4.2 双视角关联匹配器证据（2026-11-27）

状态：**Partially verified（仅合成稠密场景，离线）**。没有真实矿石、真实
标定误差或运行时验证；没有关联结果进入机器人决策链。

## 1. 目的

在 S4.1 生成器上验证 S4.2 的离线关联路径：

- 可见点反投影 + 候选 AABB/尺寸门控；
- 互相投影支持、Chamfer、可见质心 Mahalanobis；
- 匈牙利匹配与 `ambiguity_margin`；
- `associated` / `ambiguous` / `singleton` 显式输出；
- 与仅可见质心 baseline 对比。

## 2. 数据集

- 生成器：`synthetic_dual`，seed `20261127`；
- 8 个稠密场景 + 1 个空场景；
- 每场景 16–24 个椭球实例，`scene_radius=0.10 m`；
- 480×360，`ext`/`wrist` 两个固定合成相机；
- 共 165 个物理实例，**125 条构造跨视角真值对**；
- 数据集不提交，可用 `docs/ORE_SCENE_S4_2.md` 的命令重生成。

## 3. 结果

| 策略 | 预测对 | 正确 | P | R | F1 | ambiguous | forced-ID |
|---|---|---|---|---|---|---|---|
| **互相投影 + 匈牙利** | 118 | 118 | **1.000** | **0.944** | **0.971** | 0 | 0 |
| 仅可见质心 + 匈牙利 | 112 | 109 | 0.973 | 0.872 | 0.920 | 10 | 0 |

- 互相投影：precision 1.0，没有错误合并；漏 7/125 条；
- singleton 原因：`matched_elsewhere` 22、`no_candidate` 1；
- 质心 baseline：3 个错误合并、10 个 ambiguous；
- 延迟（Python，单进程，480×360，16–24 实例）：
  互相投影 p50 约 0.38 s / p95 约 0.42 s；质心 p50 约 0.36 s / p95 约 0.41 s；
- 空场景：0 cluster、0 singleton、0 错误；
- 视角顺序置换结果一致；重复运行 cluster/singleton 一致；
- `ambiguous` 路径由单元测试覆盖（对称实例构造），本数据集上互相投影为 0。

## 4. 文件

- `mutual_metrics.json`：互相投影策略的完整评测（含每场景 cluster/证据）；
- `centroid_metrics.json`：仅可见质心 baseline；
- `wrist_metrics.json`：交换首视角的置换检查；
- `summary.json`：配置、结果表、检查、限制；
- `SHA256SUMS`：本目录校验和。

## 5. 限制

1. 椭球代理 + 固定合成外参；没有真实矿石纹理/粉尘/水膜/破碎面。
2. 没有外参/时间偏移敏感性；没有真实标定残差或臂部遮挡。
3. 召回 0.944，不是“全部关联”；`matched_elsewhere`/`no_candidate`
   仍需诊断。
4. 真实矿石关联精度 **Unverified**；本证据不能作为产线精度。
5. 无 DDS/设备/运动/`jcheng-interfaces` 改动。

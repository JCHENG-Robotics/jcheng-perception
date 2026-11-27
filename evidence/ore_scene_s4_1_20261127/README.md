# S4.1 合成双视角生成器证据（2026-11-27）

状态：**Partially verified（仅生成器机制，合成数据）**。没有关联匹配器，没有
真实数据，没有硬件/运行时验证。

## 1. 目的

验证 S4.1 的离线双视角数据生成器能够稳定产生：

- 两个已标定视角（`ext` / `wrist`）的 RGB、metric z-depth、实例 label；
- 像素级遮挡（z-buffer）与每实例可见面积/遮挡比例；
- 已知构造真值的跨视角实例对（`pairs.jsonl`）；
- 空场景、确定性重生成、mask/depth/pair 一致性。

它只证明数据生成与格式正确，**不证明关联算法精度**。

## 2. 复现

```bash
cd jcheng-perception
python3 scripts/ore_scene.py generate-dual-view \
  --out /tmp/ore_s4_1_evidence/dataset \
  --scenes 4 --empty-scenes 1 --seed 20261127 \
  --width 480 --height 360 --min-instances 10 --max-instances 18
```

生成目录包含 `dataset.json`、`pairs.jsonl`、`images/`、`masks/`、`depth/`
和 `DATASET_CARD.md`。完整数据集不提交（体积与可重生成性）；本目录只保留
摘要、相机参数与两张 overlay。

## 3. 本次结果

- 4 个非空场景 + 1 个空场景；共 63 个实例；
- 跨视角真值对 **61** 个；单视角可见 singleton **2** 个；
- `ext` / `wrist` 可见实例：见 `summary.json`；
- 遮挡比例（可见面积 / 全 silhouette）中位数约 0.36 / 0.37，最大约 0.99；
- 深度范围约 0.42–0.89 m（合成相机距离）；
- 检查全部通过：确定性重生成、空场景零 mask/depth、mask 标签与 manifest
  一致、面积一致、mask 内深度为正/背景为零、pair 引用一致。

| 检查 | 结果 |
|---|---|
| 确定性重生成（同 seed/参数，逐文件 sha256） | PASS |
| 空场景 mask/depth 全零 | PASS |
| mask 标签与 manifest 一致 | PASS |
| 报告面积与 mask 像素数一致 | PASS |
| mask 内 depth>0，背景 depth=0 | PASS |
| pair 成员在双视角均可见且引用一致 | PASS |
| 测试 | **110 passed**（新增 5 个 S4.1 测试） |

## 4. 文件

- `summary.json`：生成器配置、场景/实例/对统计、检查、遮挡/深度统计、限制；
- `cameras.json`：`ext`/`wrist` 内参与 `camera_to_base`（合成外参）；
- `overlay_ext.png` / `overlay_wrist.png`：同一场景两视角的实例轮廓与编号；
- `SHA256SUMS`：本目录文件校验和。

## 5. 限制

1. 矿石用**椭球代理**，不是真实矿石几何、纹理、粉尘、水膜或破碎面。
2. 相机位姿固定且无误差；未模拟真实标定残差、热漂移、臂部遮挡。
3. 关联真值来自**构造**（同一 `global_id`），不是人工关系标注。
4. 当前没有 S4.2 匹配器；本证据不包含 association P/R/F1。
5. 不能据此声称真实矿石/产线多视角关联可用；真实域 **Unverified**。

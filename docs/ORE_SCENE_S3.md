# S3 关系图与关系基准 MVE

> 日期：2026-11-26。状态：**Partially verified（受控合成、GT mask）**。
>
> 本轮实现 v2 的 `touching`/`near` 关系边、v1→v2 关系校验，以及一个由
> **构造参数**定义真值的合成关系基准。它验证关系代码在受控场景上的正确性，
> 不是真实矿石关系精度，也不是端到端分割+关系精度。

## 1. 实现

### 1.1 关系图

`jcheng_perception_scene/relations.py`：

- `RelationParams(adjacency_px=5, touching_min_px=8, max_instances=128)`；
- `build_relation_graph(instances)`：
  - `touching`：搜索半径内 contact pixels ≥ `touching_min_px`；
  - `near`：有接触证据但低于阈值；
  - 不发 `occludes`/支撑关系：单视角 RGB 无可靠深度顺序；
  - evidence：`contact_px`、双方 boundary、contact ratio、search radius、
    `mask_iou`、`depth_order=unknown`、`directed=false`。
- `compare_relation_sets`：无序实例对 + 谓词的 P/R/F1。

### 1.2 v2 接入

`build_scene_v2(v1, geometry, relations=edges)`：

- v1 记录保持不变，v2 附加 `touching`/`near` 边；
- 校验未知实例、自环、重复边、谓词白名单；
- `validate_scene_v2` 同样检查；
- v2 schema 允许 `adjacent`/`touching`/`near`。

## 2. 关系基准

`relation_benchmark.py` 生成受控的两矩形场景：

| 类别 | 构造 | 真值 |
|---|---|---|
| `contact` | 两个 40×40 矩形，间距 0 px | 1 条 `contact` 边 |
| `near` | 间距 4 px（仍在 5 px 搜索半径内） | 1 条 `contact` 边 |
| `clear` | 间距 80 px | 无边 |

- 6 场景/类，共 18 场景；
- 真值来自**构造间距**，不是关系检测器输出；
- 评测使用数据集的 **GT mask**（oracle segmentation），把 `touching`/`near`
  折叠为 `contact` 与真值比较；
- 指标：无序实例对 + 谓词的 precision/recall/F1。

## 3. 结果

18 场景、12 条真值边、12 条预测边：

| 指标 | 值 |
|---|---|
| Precision | **1.000** |
| Recall | **1.000** |
| F1 | **1.000** |
| 每场景 | 18/18 无错误 |

这是**受控 sanity benchmark**：矩形、固定间距、GT mask。它说明关系代码在
定义清楚的场景上不产生假边/漏边，但不能外推到真实矿石堆、分割误差或真实
标注。

### 3.1 端到端预测 mask（经典 vs SAM2）

39 个合成场景（600 GT 实例）：

- GT 关系边由 GT mask 构图得到（490 条）；
- 预测实例按 mask IoU ≥0.5 与 GT 贪心匹配；
- 预测 mask 构图、映射到 GT ID 后与 GT 边比较。

| Backend | 实例匹配 | 预测/真值边 | Precision | Recall | F1 |
|---|---|---|---|---|---|
| 经典 watershed | 261/600（43.5%） | 20/490 | 0.850 | 0.035 | 0.067 |
| **SAM2.1 Tiny** | **458/600（76.3%）** | **223/490** | **0.960** | **0.437** | **0.600** |

**解读**：关系规则本身不是主要瓶颈；实例匹配率从 43.5% 提到 76.3% 后，
relation recall 从 0.035 升到 0.437，P/R/F1 从 0.067 升到 0.600。这直接量化
了“分割改善 → 关系改善”的因果。

该结果仍是合成数据，GT 边由 GT mask 推导，不是独立人工关系标注；SAM2 只在
39 张合成图上跑，不代表真实矿石。

## 4. 复现

```bash
python3 scripts/ore_scene.py generate-relation-benchmark \
  --out /tmp/relation-benchmark --per-class 6 --seed 20261126
python3 scripts/ore_scene.py evaluate-relations \
  --dataset /tmp/relation-benchmark \
  --truth /tmp/relation-benchmark/relations_truth.json \
  --out /tmp/relation-benchmark/metrics.json \
  --adjacency-px 5 --touching-min-px 8

# 端到端 predicted-mask 关系评测
python3 scripts/ore_scene.py evaluate-relations-predicted \
  --dataset <synthetic-dataset> --scenes <run>/scenes.jsonl \
  --prediction-root <run> --out <run>/relation_metrics.json \
  --adjacency-px 5 --touching-min-px 8 --match-iou 0.5
```

证据：`evidence/ore_scene_s3_relations_20261126/`（`metrics.json` 为受控
基准，`e2e_classical_metrics.json` 为经典预测 mask 端到端负结果）。

## 5. 限制

1. **合成矩形**，不是矿石；没有真实纹理、粉尘、遮挡形状。
2. **GT mask**，不是预测 mask；不包含分割误差传播。
3. `touching`/`near` 在基准中折叠为 `contact`；谓词分类精度未评估。
4. 没有独立真实关系标注；P/R/F1=1.0 不是产品精度。
5. 未做支撑面、遮挡方向、多视角关系、场景图查询。
6. 无运行时/DDS/设备/运动/接口契约改动。
7. 真实矿石关系精度 **Unverified**。

## 6. 下一步

1. 制定少量真实/合成场景的配对关系标注规范（contact / near / 不可判定）；
2. 在独立标注上报告 P/R/F1，并评估 `touching` vs `near` 谓词分类；
3. 用预测 mask（SAM2/经典）做端到端关系误差传播；
4. 再进入 S4 多视角关联。

# S4.4 敏感性/消融与预注册配置

> 日期：2026-11-27。状态：**Partially verified（仅合成混合场景，离线）**。
>
> 本轮用预注册 YAML 配置运行基线、消融、外参扰动和时间偏移敏感性。
> 结果显示深度方向外参误差显著伤关联；一条预注册验收未通过，保留为
> 失败结果，未调参掩盖。真实矿石/硬件/运行时仍未验证。

## 1. 实现

新增 `association_sensitivity.py`：

- `load_sensitivity_config(path)`：严格加载/校验预注册 YAML；
- `run_sensitivity(dataset, config, config_path)`：
  - baseline：`AssociationParams`；
  - ablations：基于 baseline 覆盖参数；
  - perturbations：对指定视角施加平移 + 绕轴旋转，重新评测；
  - time offsets：平移一视角时间戳，触发同步窗门控；
  - 输出 summary（不保存全部 scenes）与 config SHA256；
- CLI：

```bash
python3 scripts/ore_scene.py evaluate-association-sensitivity \
  --dataset <dual-view-dataset> \
  --config config/association_sensitivity.yaml \
  --out <sensitivity.json>
```

预注册配置：`config/association_sensitivity.yaml`
（SHA256 `765244867ed717d0596ab5b2e4b918d44bc171d45f5f5db94f38b18f7c79d794`）。

## 2. 数据集与配置

- 生成器 `synthetic_dual`，seed `20261127`；
- 8 个非空 + 1 个空场景，每场景 6–24 个实例，`scene_radius=0.18 m`，
  480×360；
- 130 个物理实例、122 条构造真值对；
- 同步窗 `50 ms`；外参扰动 5/10 mm、0.5/1.0°；时间偏移 0/25/100 ms；
- 消融：`full`、`no_chamfer`、`no_size`、`no_prior`、`centroid`。

## 3. 结果

### 3.1 基线

| 指标 | 值 |
|---|---|
| P / R / F1 | **1.000 / 0.934 / 0.966** |
| 真值 / 预测 / 正确 | 122 / 114 / 114 |
| ambiguous / forced-ID | 0 / 0 |
| 遮挡 低 / 中 / 高 F1 | 1.000 / 0.941 / 0.968 |

### 3.2 消融

| 变体 | F1 | ΔF1 |
|---|---|---|
| full | 0.966 | 0.000 |
| no_chamfer | 0.966 | 0.000 |
| no_size | 0.975 | +0.009 |
| no_prior | 0.966 | 0.000 |
| centroid | 0.975 | +0.009（ambiguous 3） |

Chamfer/质心先验在该合成集无贡献；尺寸门控略伤召回；仅质心略高。但
S4.2 dense 集上 centroid 明显更差（0.920 vs 0.971），且本实验只有一个
seed/一个数据集，因此**不据此改默认参数**；后续需在独立 split 重新选型。

### 3.3 外参扰动

| 扰动 | F1 | ΔF1 |
|---|---|---|
| ext tx 5 mm | 0.957 | −0.009 |
| ext tx 10 mm | 0.948 | −0.018 |
| ext tz 10 mm | **0.827** | **−0.139** |
| ext rz 0.5° | 0.966 | 0.000 |
| ext rz 1.0° | 0.966 | 0.000 |
| wrist tx 5 mm | 0.966 | 0.000 |
| wrist tz 10 mm | **0.876** | **−0.091** |

深度方向 10 mm 误差导致明显退化；横向 10 mm 尚可。绕 z 旋转 1° 在本数据
上无变化，不能外推到绕 x/y 倾斜误差。

### 3.4 时间偏移

| wrist offset | F1 | 预测对 | singleton |
|---|---|---|---|
| 0 ms | 0.966 | 114 | — |
| 25 ms（窗内） | 0.966 | 114 | — |
| 100 ms（窗外） | **0.000** | 0 | 全部 `temporal_mismatch` |

100 ms 触发 fail-closed：不关联、不输出假对；验证时间门控失败路径。

## 4. 预注册验收

| 检查 | 结果 |
|---|---|
| baseline F1 ≥ 0.90 | PASS |
| rotation 1° ΔF1 ≤ 0.10 | PASS |
| translation 10 mm ΔF1 ≤ 0.10 | **FAIL**（tz 项 −0.139 / −0.091） |
| 时间窗外 F1 = 0 | PASS |

未通过项保留，不调参掩盖。

## 5. 测试

新增 3 个测试（S4.4 结束时总计 **120 passed**）：

1. 外参扰动后的点/相机平移正确；时间戳窗外全部 `temporal_mismatch`；
2. 配置加载 + `run_sensitivity` 输出 baseline/ablation/perturbation/offset；
3. `evaluate-association-sensitivity` CLI 生成可解析结果。

## 6. 限制

1. 椭球代理、固定合成相机；无真实标定残差/热漂移/噪声。
2. 单一 seed/单一混合数据集，不是总体估计。
3. 消融为探索性；默认参数未改。
4. 只测绕 z 旋转；未测 x/y 倾斜、时间同步抖动、真实遮挡。
5. 真实矿石关联/几何精度 **Unverified**；无运行时/契约改动。

## 7. 后续

深度方向 10 mm 失败的诊断与默认关闭的校准不确定度门控见
[`ORE_SCENE_S4_4B.md`](ORE_SCENE_S4_4B.md)（2026-11-27，独立 held-out seed
上预注册 10 mm/1° 检查通过；原始失败保留，`1σ=10 mm` 不是实测标定）。

S4.5 真实静态双摄采集与人工配对子集需要单独的操作员授权；在此之前
不接真实数据、不暴露运行时。S4.6 契约提案必须在 S4.4/S4.4b 证据和 Board
评审之后另案处理。

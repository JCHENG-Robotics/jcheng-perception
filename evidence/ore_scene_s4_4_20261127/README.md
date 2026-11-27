# S4.4 敏感性/消融证据（2026-11-27）

状态：**Partially verified（仅合成混合场景，离线）**。没有真实矿石/标定误差/
运行时验证；不用于机器人决策。

## 1. 目的

用预注册配置 `config/association_sensitivity.yaml`（SHA256
`765244867ed717d0596ab5b2e4b918d44bc171d45f5f5db94f38b18f7c79d794`）在
合成混合数据集上报告：

- 基线关联指标；
- 消融（去掉 Chamfer / 尺寸 / 质心先验，以及仅质心策略）；
- 外参平移/旋转扰动（5/10 mm、0.5/1.0°）；
- 时间戳偏移（0 / 25 ms / 100 ms，同步窗 50 ms）；
- 按遮挡分层的基线指标。

## 2. 数据集

- 生成器 `synthetic_dual`，seed `20261127`；
- 8 个非空场景 + 1 个空场景；
- 每场景 6–24 个实例，`scene_radius=0.18 m`，480×360；
- 130 个物理实例、**122 条构造真值对**；
- 数据集不提交；生成命令：

```bash
python3 scripts/ore_scene.py generate-dual-view \
  --out /tmp/ore_s4_4 --scenes 8 --empty-scenes 1 --seed 20261127 \
  --width 480 --height 360 --min-instances 6 --max-instances 24 \
  --scene-radius 0.18
python3 scripts/ore_scene.py evaluate-association-sensitivity \
  --dataset /tmp/ore_s4_4 \
  --config config/association_sensitivity.yaml \
  --out /tmp/ore_s4_4/sensitivity.json
```

## 3. 基线

| 指标 | 值 |
|---|---|
| Precision / Recall / F1 | **1.000 / 0.934 / 0.966** |
| 真值 / 预测 / 正确对 | 122 / 114 / 114 |
| ambiguous | 0 |
| forced-ID 错误 | 0 |
| 按遮挡（低/中/高） | F1 1.000 / 0.941 / 0.968 |
| 空场景 | 0 cluster / 0 singleton |

按密度分档（S4.2 的 dense 集）此前为 F1 0.971；本混合集包含稀疏/中等/高
遮挡场景，因此基线略低。

## 4. 消融

| 变体 | F1 | ΔF1 | ambiguous |
|---|---|---|---|
| full | 0.966 | 0.000 | 0 |
| no_chamfer | 0.966 | 0.000 | 0 |
| no_size | 0.975 | +0.009 | 0 |
| no_prior | 0.966 | 0.000 | 0 |
| centroid | 0.975 | +0.009 | 3 |

**解读**：在该合成集上，Chamfer/质心先验没有贡献；尺寸门控略伤召回，
仅质心策略反而略高。这是**探索性发现**，不据此改默认参数：合成集只有
一个 seed，且仅质心策略在 S4.2 的 dense 集上明显更差（F1 0.920 vs 0.971）。
默认仍保留互相投影 + 全证据；后续应在独立合成/真实 split 上重新选型。

## 5. 外参扰动

| 扰动 | F1 | ΔF1 |
|---|---|---|
| ext tx 5 mm | 0.957 | −0.009 |
| ext tx 10 mm | 0.948 | −0.018 |
| ext tz 10 mm | 0.827 | **−0.139** |
| ext rz 0.5° | 0.966 | 0.000 |
| ext rz 1.0° | 0.966 | 0.000 |
| wrist tx 5 mm | 0.966 | 0.000 |
| wrist tz 10 mm | 0.876 | **−0.091** |

- 横向平移 10 mm 可接受（ΔF1 −0.018）；
- **深度方向 10 mm 误差导致明显退化**（extern −0.139、wrist −0.091）；
  这比横向误差更危险，因为会改变互相投影的深度一致性。
- 绕 z 轴旋转 1° 在本数据上无变化；不能外推到绕 x/y 的倾斜误差。
- 预注册验收 `max_translation_10mm_f1_drop ≤ 0.10` **未通过**（因 tz 项），
  作为失败结果保留，不调参掩盖。

## 6. 时间偏移

| wrist offset | F1 | 预测对 | singleton |
|---|---|---|---|
| 0 ms | 0.966 | 114 | — |
| 25 ms（窗内） | 0.966 | 114 | — |
| 100 ms（窗外） | **0.000** | 0 | 全部 `temporal_mismatch` |

100 ms 偏移触发 fail-closed：不关联、不输出假对。该实验验证时间门控的
失败路径，不是运动导致的精度退化。

## 7. 检查与限制

| 检查 | 结果 |
|---|---|
| baseline F1 ≥ 0.90 | PASS |
| rotation 1° ΔF1 ≤ 0.10 | PASS |
| translation 10 mm ΔF1 ≤ 0.10 | **FAIL**（tz 项） |
| 时间窗外 F1 = 0 | PASS |
| 测试 | **120 passed**（新增 3 个 S4.4 测试） |

限制：

1. 椭球代理、固定合成相机；没有真实标定残差/热漂移/噪声。
2. 单一 seed、单一混合数据集；不是总体估计。
3. 消融结果仅探索性；不据此改默认，避免在测试集上调参。
4. 绕 z 旋转无变化不代表倾斜误差无害；未测 x/y 轴旋转。
5. 真实矿石关联/几何精度 **Unverified**；无运行时/契约改动。

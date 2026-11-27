# S4.4b 校准不确定度深度门控证据（2026-11-27）

状态：**Partially verified（独立合成 held-out seed，离线）**。没有真实矿石/
标定残差/运行时验证；不用于机器人决策。原始 S4.4 的 10 mm 深度方向失败
结果原样保留，本文件不追溯改写该失败。

## 1. 目的

S4.4 预注册验收失败于外参深度方向 10 mm 误差：F1 从 0.966 降至
0.827（ext）/ 0.876（wrist）。诊断显示机制是互相投影的深度一致性门控
`z <= target_depth + depth_tolerance_m` 把整片投影点判为“被遮挡”，分母
归零，`symmetric_support=0`，大量真值对落入 `insufficient_visible`。

S4.4b 验证一个最小、默认关闭的修正：把**已声明的外参深度不确定度**传播
到深度门控，而不是调大固定阈值。

```text
effective_depth_tolerance_m =
    depth_tolerance_m + depth_sigma_multiplier * extrinsic_depth_sigma_m
```

- 新增参数 `extrinsic_depth_sigma_m`（默认 `0.0`）与
  `depth_sigma_multiplier`（默认 `2.0`）；
- 默认 `sigma=0` 时有效门限与旧行为完全相同，历史结果/测试不改变；
- 本轮预注册假设：深度方向外参不确定度 `1σ = 10 mm`，取 `2σ` 包络，
  有效门限 `0.01 + 2 × 0.010 = 0.030 m`；
- 只改深度门控；像素横向偏移、Chamfer、尺寸、质心先验均不变。

## 2. 预注册与 held-out

| 项 | 值 |
|---|---|
| 配置 | `config/association_sensitivity_calibrated.yaml` |
| 配置 SHA-256 | `71ad904a7e42f7687e09d5d442921ca05ff9797d9fa59438224a56a4414bd575` |
| 冻结时点 | 生成 held-out 数据**之前**；生成后未改配置 |
| held-out seed | `20261128`（与 S4.4 的 `20261127` 不同） |
| 基线参数 | `depth_tolerance_m=0.01`，`sigma=0.010`，`multiplier=2.0` |

S4.4 的 uninformed 失败（seed `20261127`）仍是历史证据；S4.4b 是新的
校准感知配置在独立 seed 上的一次性预注册验收，不是对旧结果的重新解释。

## 3. 数据集

- 生成器 `synthetic_dual`，seed `20261128`，版本 `1.0.0`；
- 8 个非空 + 1 个空场景，480×360；
- 每场景 6–24 实例，`scene_radius=0.18 m`；
- 128 个物理实例、**126 条构造真值对**；
- 数据集不提交；生成命令见 §7。

## 4. 基线（校准感知）

| 指标 | 值 |
|---|---|
| Precision / Recall / F1 | **1.000 / 0.952 / 0.976** |
| 真值 / 预测 / 正确对 | 126 / 120 / 120 |
| ambiguous / forced-ID 错误 | 0 / 0 |
| singleton 原因 | `matched_elsewhere` 12，`size_gate` 1 |
| 按密度 dense / medium / sparse | F1 0.972 / 1.000 / 1.000 |
| 按遮挡 high / medium / low | F1 0.974 / 0.974 / 1.000 |
| 延迟 P50 / P95 | 319.9 / 397.3 ms（9 场景，单进程） |

与 S4.4 的 seed `20261127` 基线（F1 0.966）不是同一数据集，不能直接当
改进量；这里只作为本 held-out 的基线。

## 5. 预注册验收（held-out）

| 检查 | 结果 |
|---|---|
| baseline F1 ≥ 0.90 | **PASS**（0.976） |
| 10 mm 平移 ΔF1 ≤ 0.10 | **PASS**（最差 `wrist_ty_10mm` ΔF1 0.060） |
| 1° 旋转 ΔF1 ≤ 0.10 | **PASS**（六个轴均 ≤0.004） |
| 时间窗外 F1 = 0 | **PASS**（100 ms：F1 0，253 个 `temporal_mismatch`） |

### 5.1 平移扰动

| 扰动 | F1 | ΔF1 | P | R |
|---|---|---|---|---|
| ext tx 5 mm | 0.971 | −0.004 | 1.000 | 0.944 |
| ext tx 10 mm | 0.959 | −0.017 | 1.000 | 0.921 |
| ext ty 10 mm | 0.971 | −0.004 | 1.000 | 0.944 |
| ext tz 5 mm | 0.963 | −0.013 | 1.000 | 0.929 |
| ext tz 10 mm | 0.946 | −0.030 | 1.000 | 0.897 |
| wrist tx 10 mm | 0.967 | −0.008 | 1.000 | 0.937 |
| wrist ty 10 mm | **0.916** | **−0.060** | 0.973 | 0.865 |
| wrist tz 10 mm | 0.950 | −0.026 | 1.000 | 0.905 |

### 5.2 旋转扰动（1.0°）

| 扰动 | F1 | ΔF1 |
|---|---|---|
| ext rx / ry / rz | 0.976 / 0.976 / 0.976 | 0.000 / 0.000 / 0.000 |
| wrist rx / ry / rz | 0.971 / 0.976 / 0.971 | −0.004 / 0.000 / −0.004 |

绕 x/y 的倾斜误差在本数据上同样没有造成退化；这弥补了 S4.4 只测绕 z 的
覆盖缺口，但仍只代表本合成几何。

### 5.3 时间偏移

| wrist offset | F1 | singleton |
|---|---|---|
| 0 ms | 0.976 | `matched_elsewhere` 12，`size_gate` 1 |
| 25 ms（窗内） | 0.976 | 同上 |
| 100 ms（窗外） | **0.000** | 全部 `temporal_mismatch` |

## 6. 事后诊断对照（不参与验收）

在 held-out 数据上，对同一批 10 mm 扰动比较 `sigma=0`（未声明不确定度）
与 `sigma=10 mm / 2σ`（本轮配置）。这是预注册验收跑完之后的事后诊断，
只用于量化门控传播的贡献，不改变验收结论。

| 扰动 | sigma=0 F1 | sigma=10 mm F1 | 恢复 ΔF1 |
|---|---|---|---|
| ext tx 10 mm | 0.959 | 0.959 | 0.000 |
| ext ty 10 mm | 0.967 | 0.971 | +0.004 |
| ext tz 10 mm | **0.875** | 0.946 | **+0.071** |
| wrist tx 10 mm | 0.967 | 0.967 | 0.000 |
| wrist ty 10 mm | 0.916 | 0.916 | 0.000 |
| wrist tz 10 mm | **0.880** | 0.950 | **+0.070** |

事后对照复现了 S4.4 的机制：未声明不确定度时，深度方向 10 mm 使 ext tz
F1 降至 0.875（相对本 held-out 基线下降 0.101，超过 0.10 门槛），而
校准感知配置恢复到 0.946。`wrist_ty` 的 0.060 降幅与深度门控无关（横向
扰动，sigma 不改变结果），属于像素/证据项的残余损失。

## 7. 文件与复现

- `config.yaml`：预注册配置副本；
- `sensitivity.json`：held-out 一次性运行原始输出；
- `summary.json`：紧凑摘要、验收、事后对照与限制；
- `posthoc_uninformed.json`：事后 `sigma=0` 对照原始数值；
- `SHA256SUMS`：本目录校验和。

复现命令：

```bash
python3 scripts/ore_scene.py generate-dual-view \
  --out /tmp/ore_s4_4b --scenes 8 --empty-scenes 1 --seed 20261128 \
  --width 480 --height 360 --min-instances 6 --max-instances 24 \
  --scene-radius 0.18
python3 scripts/ore_scene.py evaluate-association-sensitivity \
  --dataset /tmp/ore_s4_4b \
  --config config/association_sensitivity_calibrated.yaml \
  --out /tmp/ore_s4_4b/sensitivity.json
```

## 8. 测试

| 项 | 结果 |
|---|---|
| 全量测试 | **122 passed**（S4.4 结束时 120，新增 2 个 S4.4b 测试） |
| 新测试 1 | 有效深度门限计算与负 sigma/负 multiplier 拒绝 |
| 新测试 2 | 15 mm 深度偏差在默认门限下支持为 0，在 2σ=10 mm 配置下恢复为 1.0 |

## 9. 限制

1. 椭球代理、固定合成相机；没有真实标定残差、热漂移、深度噪声或粉尘。
2. 单一 held-out seed；不是总体估计。
3. 声明的 `1σ=10 mm` 是测试包络假设，不是实测标定协方差；真实系统必须
   由标定流程给出 `sigma` 并写入配置。
4. 只验证了深度门控传播；横向像素偏移、遮挡边界和 Chamfer 项的残余误差
   仍在（`wrist_ty` ΔF1 0.060）。
5. 真实矿石关联/几何精度 **Unverified**；无 DDS/设备/运动/契约改动。
6. 事后对照不是预注册验收的一部分，不能替代独立复现。

# 融合、关联与滤波

## 1. 测量模型

一个检测框 + 时间匹配的深度帧 → 相机光学系下：

```text
p_cam = [ (u-cx)·z/fx, (v-cy)·z/fy, z ]
p_base = T_base_cam(t_depth) · p_cam
```

每轴 1σ：

```text
σ_x = z · σ_px / fx
σ_y = z · σ_px / fy
σ_z = sqrt( (q · z²)² + floor² )
```

- `σ_px` 默认 1.5 px；关键点检测按其置信度放大：`σ_px / max(conf, 0.15)`。
- `q` 是距离二次项（米/米²），`floor` 是近距底噪。默认值：
  - D435i：`q=0.0025`，`floor=0.0015 m`（≈0.1 px 视差、50 mm 基线）；
  - Aurora：`q=0.015`，`floor=0.0020 m`（初始值，需实机噪声扫描）。
- 相机系协方差 `C_cam = diag(σ_x², σ_y², σ_z²)`；转到目标系
  `C = R_base_cam · C_cam · R_base_camᵀ`。
- 关键点优先于框中心；框中心无效时在内框 3×3 网格取有效深度中位数。
  深度为空洞/超量程/尺寸不匹配 → **丢弃**，不猜。

## 2. 视图质量

```text
quality_i = score
          · (1 + 0.5 · keypoint_score)
          · (1 - 0.75 · occlusion_ratio)
          · range_penalty(z)
```

- `score`：检测置信度，最低按 0.05 计。
- `keypoint_score`：关键点模型给的支持度；有键点检测时提升。
- `occlusion_ratio`：框内有效深度比例不足（<50%）与触边比例的较大者；
  关键点命中时减半。
- `range_penalty`：超出相机量程时高斯衰减，最低 0.05：

```text
z > z_max: exp(-((z-z_max)/width)²)
z < z_min: exp(-((z_min-z)/width)²)
否则 1
```

## 3. 动态权重

```text
factor_i = prior(phase, role) · quality_i / σ_eff_i²
σ_eff_i² = trace(C_i) / 3
```

`prior` 是相位/角色先验（相对信息倍数）：

| 相位 | external | wrist | 场景 |
|---|---|---|---|
| SEARCH | 1.0 | 1.0 | 搜索，任一视图 |
| COARSE | 1.3 | 0.6 | 外机位粗定位 |
| ACQUIRE | 1.0 | 1.0 | 两视图建立同一目标 |
| FINE | 0.35 | 3.0 | 腕相机精细抓取 |
| DROP_RECHECK | 3.0 | 0.0 | 外机位落点复检；腕相机不参与 |

**按距离动态加权**来自 `σ_eff² ∝ z²` 与 range penalty，而不是额外的距离启发式：
近处视图方差小，自然获得更高信息权重；先验只决定“同等精度时谁主导”。

归一化与融合：

```text
factor_i ← factor_i / max(factor)
w_i = factor_i / Σ factor_j
Ω = Σ factor_i · C_i⁻¹          (信息矩阵)
R_fused = Ω⁻¹
p_fused = Σ w_i · p_i
```

- 单视图时 `max(factor)` 归一化为 1，`R_fused = C`，不会人为“变准”。
- 两个等精度视图 → `R_fused = C/2`。
- `min_view_weight=0.02`：权重过低的视图被剔除，避免极端先验下的数值噪声。
- `FusedTarget.weight_external/wrist` 是归一化权重，便于日志和调参。

### 3.1 实测示例（合成场景）

- 外机位深度 ≈ 0.23 m，腕相机 ≈ 0.36 m。
- FINE 先验 0.35 / 3.0，`σ_eff` 外机位更小但腕先验更高。
- 实测融合权重：**external 0.21 / wrist 0.79**；相位 FINE。

## 4. 关联

- 代价：观测与轨迹预测位置的 3D 欧氏距离。
- 门限：

```text
gate = gate_base + gate_growth · (1 - exp(-age/0.35)) + gate_per_s · age
默认 0.08 + 0.12·(...) + 0.10·age  (米)
```

- 标签必须一致（`association_label_agnostic: false`）。同一套相机模型若类别名
  不同，可开启 agnostic，但会牺牲多类区分。
- 每个轨迹在**同一次 flush 内每台相机最多接收一个观测**，避免同一相机的两个
  框塌缩到一条轨迹。
- 未匹配观测新建 `TENTATIVE` 轨迹；命中 `confirm_hits` 次后 `CONFIRMED`。

## 5. 轨迹生命周期

| 状态 | 条件 | 输出 |
|---|---|---|
| TENTATIVE | 新建，命中 < confirm_hits | 默认不发布 |
| CONFIRMED | 命中 ≥ confirm_hits | 发布 |
| COASTING | 距上次更新 > coast_after_s（0.6 s） | 继续预测，置信度衰减 |
| LOST | > lost_after_s（1.8 s） | 不发布；2.3 s 后删除 |

- 预测：恒定速度模型，位置 `p + v·dt`。
- 置信度衰减：`exp(-(age-0.25)/0.8)`，避免“僵尸目标”被当作新鲜。
- 协方差随 `q_a·dt³/3` 增长；超过 `max_position_variance_m2` 的目标不发布。

## 6. 卡尔曼滤波

状态 `[x,y,z,vx,vy,vz]`，恒定速度：

```text
F = [[I, dt·I],
     [0,   I ]]
Q = q_a · [[dt³/3·I, dt²/2·I],
           [dt²/2·I, dt·I ]]
```

- `q_a = 0.05 m²/s³`（桌面慢速目标；从 0.35 下调，避免检测间隔内协方差膨胀）。
- 观测矩阵只取位置；Joseph 形式更新保持协方差对称正定。
- 创新协方差含测量噪声 + `measurement_floor²`。
- 双视图在同一合并窗口内时，用加权融合后的 `p_fused/R_fused` 做一次更新；
  两视图分时到达时，先到者先更新，后到者再更新（信息不丢失，顺序略有差异）。

## 7. 重复轨迹合并

启动瞬态或短暂漏关联会产生“每相机一条”的重复轨迹。之后每台相机都会优先
匹配自己那条（距离 0），普通关联永远不会合并。`merge_duplicates()` 每帧执行：

| 条件 | 门限 |
|---|---|
| 相机集合**不相交**（外机位一条、腕相机一条） | 6 cm 或 3σ 的较大者 |
| 相机集合**有交集**（重复轨迹已被两视图分别更新） | **严格 2 cm** |

- 标签必须一致（或开启 agnostic）。
- 保留命中多/更老的轨迹；吸收对方的状态（预测到较新时间后做一次 KF 更新）、
  命中计数、腕/外机位命中数、方向。
- 同相机两条明显分离的轨迹不会合并（2 cm 是分辨率下限，不是目标间距假设）。

## 8. 方向估计

两种来源，按优先级：

1. **关键点轴**：`keypoint_axis_pairs` 指定的两个键点，深度反投影到 base，
   取水平方向角；σ ≈ `min(0.30, 0.05/轴长)` rad。
2. **点云主方向**：框内深度点拟合桌面（SVD），保留高于桌面 8 mm 的点，做水平
   PCA，主轴角为 yaw；σ 随支撑比例增大，最小 0.12 rad。

限制：

- 矩形/圆柱目标的 yaw 存在 **±90°/180° 歧义**；抓取规划需要夹爪对称性容忍，
  或用键点/物体模型消歧。
- 输出协方差中 roll/pitch 标记为未观测（1.0），不要当作有效方向。
- 平面目标（贴纸、薄片）没有高于桌面的点，方向为空。

## 9. 调参顺序

1. 用实机静态目标测 `depth` 噪声 vs 距离，拟合 `q`、`floor`（每相机）。
2. 用固定基座标定验证 `T_base_camera`，确认外机位与腕相机在同一目标上的偏差。
3. 调 `association_gate_m`：目标运动速度 × 检测间隔 + 3σ 测量误差。
4. 调 `merge_gate_*`：从保守开始，观察是否出现错误合并。
5. 调相位先验：先让 `FINE` 腕权重接近 0.8；再检查 `COARSE`/`DROP_RECHECK`。
6. 最后调 `kalman_accel_noise`：太大协方差膨胀，太小跟踪滞后。

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

- 预测：恒定速度模型，位置 `p + v·dt`，`dt` 限幅 `max_extrapolation_s`
  （0.35 s，见 §9.3）。
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
- 位置更新**不改速度状态**（`velocity_gain=0`）；速度由同一相机相邻帧的差分
  作为独立量测走 `update_velocity()`。跨视图差值是测量点偏差，不能当速度，
  详见 §9.3。
- 双视图在同一合并窗口内时，用加权融合后的 `p_fused/R_fused` 做一次位置更新；
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

## 9. 跨视角偏差与轨迹漂移

### 9.1 测量模型

每个视图测到的是**不同的物理点**（外机位的框中心落在杯子侧面，腕相机的框中心
落在杯口/杯顶），相机链本身还有外参/深度偏差。观测模型写成：

```text
z_i(t) = x(t) + b_i + m_i + n_i(t)
```

- `b_i`：相机链的系统偏差（外参、深度偏置），对相机恒定；
- `m_i`：被测物体的**测量点偏差**（框中心相对任务参考点），对“物体 + 视图”
  在慢速运动下近似恒定；
- `n_i`：零均值噪声，协方差来自 §1。

两次视图只能观测到 `c_i = b_i + m_i` 的差；给所有 `c_i` 同加一个常数不会改变
任何可测量。因此**以腕相机为参考**（`c_wrist = 0`），只估计外机位的相对量。
融合输出的绝对位置因此是“腕相机参考点”，不是几何中心；在换用固定点
（AprilTag/键点）之前，不要引用绝对精度。

### 9.2 两层估计

| 层 | 内容 | 估计方式 | 存放 |
|---|---|---|---|
| 链级 | `b_ext` | 固定点/桌面平面离线标定 | `config/view_bias.yaml` |
| 物体级 | `m_ext − m_wrist` 残差 | 每 track 在线学习 | `Track.view_offset` |

- `config/view_bias.yaml` 的 `offset_m` 是**加到**反投影位置上的修正量，
  `sigma_m` 是修正量的 1σ，按平方和加到测量协方差，避免“临时修正”被当成
  毫米级精确观测。当前外部值来自 2026-11-24 run 3：wrist − external =
  (+9.9, −1.8, +9.4) mm（depth）/ (+9.0, ±2, +5.2) mm（PnP），桌面平面
  外机位低 10.7 mm；取 6 mm σ 作为保守不确定度。
- 每 track 的残差用**时间邻近的跨视图样本**学习：同一批合并窗口内的 ext/wrist
  优先；否则使用该 track 保存的、捕获时间差 ≤ `view_offset_pair_max_age_s`
  的最近异视图样本。第一对直接初始化，之后按 `view_offset_alpha` 指数跟随。
  这样两视图异步到达、30 ms/600 ms 合并窗口漏配时仍能收敛。
- 单视图外机位更新把 `view_offset` 的剩余不确定度按
  `max(view_offset_sigma_m/√hits, view_offset_floor_sigma_m)` 加到协方差；
  同一次更新里有配对样本时不加（该更新自身约束了偏差）。

### 9.3 速度只能来自同视图差分

跨视图差值是测量点偏差，不是运动。因此：

- 位置更新使用 `velocity_gain = 0`：KF 的位置行照常更新，速度状态不被
  “换视图的 60 mm 跳变”污染；
- 速度用**同一相机相邻两帧**的差分作为独立量测，经协方差加权后走
  `update_velocity()`（`H = [0 | I]` 的 Joseph 更新）。每视图的 `b_i + m_i`
  在差分中抵消；基线超过 `velocity_sample_max_age_s` 或短于
  `velocity_min_dt_s = 0.10 s` 则丢弃。后者是实机 tag 数据发现的：同一
  capture 的重复/近似重复观测 dt 可到 0，1 mm 差除以 0.04 s 会伪造 ~25 mm/s。
- 初速度先验从 1 m/s 收紧到 `initial_velocity_sigma_mps = 0.25 m/s`；
  发布与关联预测的 `p + v·dt` 中 `dt ≤ max_extrapolation_s = 0.35 s`。
  毫米级测量噪声估出的速度若积分 1 s 就是分米级漂移，这是 run-7 的直接放大器。

### 9.4 回归证据（2026-11-24，离线重放 run-7 观测）

`scripts/replay_static_drift.py` 用录制的 223 条观测重放（测量协方差按
`cameras.yaml` 重建，位置/时间/标签用录制值）：

| 数据 | 静止目标最大漂移 |
|---|---:|
| 实机录制发布值（修复前） | cup-005 696 mm，dining_table-035 448 mm |
| 离线重放，旧跟踪逻辑 | required 标签最差 1444 mm |
| 离线重放，本文模型 | required 标签最差 **6.4 mm**（cup 4.1 / table 4.9 / chair 6.4 / person 4.4） |

学到的每 track 残差与离线统计一致：cup (−57.2, +10.9, +34.0) mm（扣掉全局
修正后），dining table (+22.1, +21.1, −5.0) mm。kite 未纳入门限：同一标签在
两只物理风筝/腕部检测跳动之间切换，属关联/数据问题，不是偏差模型能修的。
证据：`evidence/analysis_20261124_drift_bias/`。

### 9.5 限制

- 该模型修的是**静态/慢速**目标的漂移；两视图捕获时刻差内的真实运动会被
  部分吸收进 `view_offset`，快速运动时偏差估计会滞后。要 <5 mm 的绝对精度，
  仍需固定点/键点（独立任务）。
- `m_i` 与物体形状、姿态有关；换物体后第一对样本会重新初始化残差，
  期间位置可能在两视图参考点之间摆动，但不会再触发速度漂移。
- 绝对位置以腕相机为参考；不要用该输出去校准外部链的绝对精度。

### 9.6 参考点模式（AprilTag / 键点）

§9.2 的对象级 `m_i` 是 bbox 中心落在物体上的位置，换视图就差几十毫米。要消掉
它，需要一个**两个视图测的是同一个物理点**的定义点：

- **固定点（当前实现）**：AprilTag 中心。GPU 新增 `aruco` 后端
  （`config/gpu_tag.yaml`，`DICT_APRILTAG_36h11`、60 mm、id 0），输出标签
  `reference_tag_<id>`：关键点 0 = 标签中心（置信度 1.0），1–4 = 四角（0.9），
  框为标签外接框。
- Edge **不需要改动**：现有 `build_observation` 优先用最高置信度关键点的深度，
  标签中心就是 3-D 点；关键点深度无效时才回退到框中心。
- 两个视图的中心是同一物理点，`Track.view_offset` 只剩链级偏差 + 深度/PnP
  偏置（mm 级），首个跨视图配对的参考点跳变也从 57–67 mm 降到 mm 级
  （`tests/test_view_bias.py::test_reference_point_keeps_the_first_pair_step_small`）。
- 限制：打印标签纸会翘曲（实测最大 ~7 mm），做绝对精度前先压平；depth 与
  RGB-only PnP 之间存在 ~4 mm 的每视图偏差，仍要用
  `scripts/fixed_point_check.py` 的 PnP/depth 两条路径做绝对参考。
- **键点模型**是同一机制：一旦有 pose/keypoint 权重，把抓取点作为中心键点输出，
  融合/偏差层不需要再改；当前厂商树没有可用的关键点权重，所以先用标签。

验证方法：GPU 用 `gpu_tag.yaml`，edge 仍用 `fusion_dual_hardware.yaml` +
`view_bias.yaml`，静置 60 s；检查两视图 `reference_tag_0` 的位置差、首个配对
跳变和 FUSED 漂移，再用 `fixed_point_check.py` 给绝对参考。

**实机结果（2026-11-24，唯一一次双视图成功采集）：**

- 两视图标签中心差 **3.77 mm**（external − wrist = +0.2/−2.0/−3.2 mm，
  每视图 σ ≤ 0.9 mm）；`Track.view_offset` 学到 (+0.03, −1.93, −3.31) mm。
- 固定点 5 mm gate：未加链级修正 15.92 mm（depth）/12.79 mm（PnP）FAIL；
  加 `view_bias.yaml` 后 **3.67 / 3.07 mm PASS**。这是同一链级修正下的一致性
  核验，不是独立绝对真值。每视图 depth-vs-PnP 仍有 4.2 mm（外机位）/7.4 mm
  （腕部）偏差，来自标签纸翘曲与斜视。
- 没有 67 mm 首配对跳变：标签轨迹从外机位点起步，只按 3.8 mm 残差切到腕部
  参考；融合轨迹最大 11.4 mm、p95 6.2 mm、最终 3.7 mm（实机，eed1ca9）。
- 标签数据暴露了另一个问题：47/191 条腕部观测的 capture 间隔 ≤0，同视图差分
  除以 0.04 s 会伪造 ~25 mm/s 速度，外推 0.35 s 就是 10 mm 跳变。
  `velocity_min_dt_s = 0.10` 拒绝这种基线；重放降到 **8.3 mm**（p95 5.4）。
  该修正尚未部署到 Pi，实机 11.4 mm 是修正前的值。
- 绝对参考（腕部 PnP）：(335.4, −12.7, −85.7) mm；修正后外机位 PnP
  (334.7, −14.7, −83.5) mm；两者 3.07 mm。桌面参考 −82.5 mm，标签中心在
  桌面上方约 3 mm（纸翘曲，不是平面）。

证据：`evidence/hardware_20261124_tag_verify/`。限制：单一静态姿态、单个标签；
先压平标签纸；GPU 外机位读端仍会 wedge（本次“先 edge、再重启 GPU”才恢复）。

**min-dt 复跑（f90b5d5，部分确认）：** 部署后 60 s 复跑只得到外机位 45 条 tag
观测（t=0.3–14.3 s），腕部 depth matching 失败 365 次、GPU 外机位流约 14 s 后
停止；融合轨迹 6 s 后 **0.4 mm**（max 2.42 mm），说明 min-dt 不引入漂移，但
**没有取得双视图复跑**。第二次复跑（Aurora 驱动重启 + GPU 每 10 s 重启的
harness）把腕部 depth failures 从 365 降到 **1**，但 edge 在约 6 s 后帧摄入
停止（88 帧、27 detections、gpu_alive=false），外机位-only 轨迹 1.8 mm；仍未
取得双视图。两次复跑都指向 **edge 帧摄入 / Pi 负载**这个剩余阻塞，而不是
min-dt。双视图定义点结果仍以 run 3 为准。

**Humble 同发行版复跑（run 6）：** 发现 laptop 有 Humble 容器后，用
`armpi-humble-moveit` 新建 `jcheng-humble-tag`（host 网络 + wired CycloneDDS
配置），装 `python3-opencv` 4.5.4（含 aruco），在 `/workspace/tag_ws` 构建
msgs/core/gpu；laptop GPU 与 Pi edge 两端同为 **Humble**。同时修了 edge uplink：

- `uplink_allowed()` 在时钟回跳时重置每相机 uplink 基线；否则 `last` 留在
  未来，该相机 uplink 会被永久跳过。测试：`tests/test_uplink_rate.py`。
- `UplinkWorker` 捕获任意 encode/publish 异常并继续，不再静默退出；status
  增加 `encode_failures` / `rate_limited` 计数。

60 s 结果：141 条腕部 tag 观测（0.5–59.8 s），edge 468 帧 / 410 detections /
234 matched；**腕部-only** 融合轨迹 max **7.30 mm**、p95 4.66、final 3.93 mm
（另一早期轨迹 10.03 mm）。这达到 ~8 mm 目标，但**外机位仍 0 条 tag 观测**：
GPU 只收到 42 外机位帧（腕部 376），12 s 检查无外机位 uplink 消息；当前外机位
画面里标签和 0.225 m 深度都正常，所以仍是外机位 uplink/reader 问题，不是模型
或发行版。双视图确认仍未完成。

**订阅 watchdog 诊断（已回退）：** 加过“某相机 frames/depth 计数 4 s 不增长就
销毁并重建其 image/depth/CameraInfo 订阅”的 watchdog。实测外机位 **RGB 卡在
5 帧**（depth 仍 31→75→117），腕部 **depth 卡在 7**（RGB 40→80→123），重建 3
次都不恢复。wedge 在 **DDS participant/reader 层**，不是订阅对象；同进程重建
无效。watchdog 已回退，Pi 只保留 committed 的 `2a33cbb` uplink 钟跳/线程修复。
下一步应做 reader 隔离（每相机独立 participant/relay）或 CycloneDDS 版本/配置
修正。

**网关根因与双视图 run 9（2026-11-24）：** 最终定位到两个**网关侧消费者**，
不是感知代码：

1. `armpi-gateway.service`（8765，aux 预览）没有传 `RMW_IMPLEMENTATION` /
   `CYCLONEDDS_URI`，默认 Fast DDS，订阅 `/camera/color/image_raw`；大彩色
   样本跨 RMW，导致 edge 外机位 RGB reader wedge。改启动脚本/`gateway.env`
   显式 CycloneDDS 后，外机位连续 60 s。
2. 8766 motion 网关是一个 **PPID=0 的孤儿进程**（service 已 inactive），仍订阅
   `/depth_cam/depth_registered/points`，阻塞 Aurora 的 depth 发布；kill 后腕部
   depth reader 恢复。

Run 9（双视图）：external 240 + wrist 195 条，跨视图 **3.37 mm**；融合轨迹
max **10.03 mm**、p95 **3.78 mm**、final **1.19 mm**（重放 9.9/3.6/1.4，
学到的 view_offset (+0.48,−1.21,−2.98) mm）。max 来自 t=26.2 s 一次 12.4 mm
的**视图切换瞬态**；p95/final 均在 ~8 mm 内，且**没有 67 mm 参考切换**。
注意：motion 网关当前已停；若为 Studio 运动恢复，需先处理其点云订阅
（QoS/消费者），否则腕部 depth 会再次被阻塞。

## 10. 调参顺序

1. 用实机静态目标测 `depth` 噪声 vs 距离，拟合 `q`、`floor`（每相机）。
2. 用固定基座标定验证 `T_base_camera`，确认外机位与腕相机在同一目标上的偏差。
3. 调 `association_gate_m`：目标运动速度 × 检测间隔 + 3σ 测量误差。
4. 调 `merge_gate_*`：从保守开始，观察是否出现错误合并。
5. 调相位先验：先让 `FINE` 腕权重接近 0.8；再检查 `COARSE`/`DROP_RECHECK`。
6. 最后调 `kalman_accel_noise`：太大协方差膨胀，太小跟踪滞后。

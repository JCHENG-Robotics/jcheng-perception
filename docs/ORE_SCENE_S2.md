# S2 可见几何 MVE 报告

> 日期：2026-11-26。状态：**Partially verified（仅离线合成深度）**。
>
> 本轮只实现 mask → **可见**点云/可见中心/协方差/AABB/OBB 的几何数学与失败
> 路径。没有真实深度、没有真机、没有 schema v2；真实矿石几何精度 **Unverified**。

## 1. 目标与非目标

### 目标

给定一个实例 mask、同尺寸的米制深度、相机内参和 `base <- camera` 4×4 变换，
输出：

- 可见点数量、深度有效率；
- 可见点云中心（`visible_centroid_base`）；
- 可见中心的 3×3 样本协方差；
- 可见 AABB、PCA OBB（中心、四元数、尺寸）；
- 明确的 `status`、`reason`、`unknowns`。

### 非目标

- 不输出真实质心、完整尺寸、体积、质量、材质、品位或价值。
- 不把可见中心当抓取位姿。
- 不做 table plane/支撑面、遮挡方向、多视角关联。
- 不接运行时/DDS/设备/运动。

## 2. 接口

```python
from jcheng_perception_scene.geometry import GeometryParams, visible_geometry

result = visible_geometry(
    mask,                  # bool HxW
    depth_m,               # float HxW, metric, aligned to mask
    camera_matrix,         # 3x3 pinhole K
    transform_base_camera, # 4x4 base <- camera optical
    GeometryParams(stride=2, min_depth_m=0.05, max_depth_m=2.0,
                   min_points=20),
    frame_id='base_link',
)
```

约束：

- `depth_m` 必须已经对齐到 mask，且单位是米；不接收 16UC1，不在这里做
  `depth_scale`。
- 深度无效不插值、不补洞；有效点不足 `min_points` 即
  `status='insufficient_depth'`。
- mask 为空 → `status='invalid_mask'`。
- 形状不一致 / 非 3×3 K / 非 4×4 TF → `ValueError`（fail closed）。

## 3. 输出字段

| 字段 | 含义 |
|---|---|
| `status` | `computed` / `insufficient_depth` / `invalid_mask` |
| `reason` | 失败原因或 null |
| `frame_id` | 输出坐标系（默认 `base_link`） |
| `visible_point_count` | stride 采样后的有效点数 |
| `depth_valid_fraction` | 采样点中有效深度比例 |
| `visible_centroid_base` | 可见点云均值，**不是真实质心** |
| `visible_centroid_covariance` | 3×3 样本协方差；未校准，不是测量不确定度 |
| `visible_aabb` | `center` / `extent` / `frame_id`，base 轴对齐 |
| `visible_obb` | `center` / `quaternion_xyzw` / `extent` / `axes` / `frame_id` |
| `field_status` | 可见字段 `algorithm_estimate`；真实质心/完整尺寸/体积 `unknown` |
| `unknowns` | `true_centroid`、`full_extent`、`volume`、`mass`、`material_composition`、`ore_grade`、`economic_value` |

## 4. 计算

```text
x_cam = (u - cx) * z / fx
y_cam = (v - cy) * z / fy
z_cam = z
p_base = R_base_cam * p_cam + t_base_cam

visible_centroid_base = mean(p_base)
covariance = cov(p_base, rowvar=False, ddof=1)
AABB = [min(p_base), max(p_base)] in base axes
OBB  = PCA(covariance) -> axes, project p_base, center/extent, quaternion
```

OBB 的轴符号选择保证右手系；`quaternion_xyzw` 与 core 的
`quaternion_from_matrix` 一致。平面点云（z 方差为 0）仍可得到 OBB，但其法向
轴上的 extent 为 0，不能据此推断物体厚度。

## 5. 测试结果（合成深度）

`tests/test_scene_pipeline.py` 新增 2 个几何测试：

| 用例 | 结果 |
|---|---|
| 480×640、深度恒 1.0 m、100×100 居中 mask、stride 2 | 2500 点；可见中心 z=1.0；AABB extent ≈ 0.196×0.196×0；OBB 非空；协方差 3×3 |
| base←camera 平移 [0.1,0.2,0.3] | 可见中心 ≈ [0.1,0.2,1.3] |
| 深度全 0 | `insufficient_depth`；中心/AABB/OBB 为 null；有效率 0 |
| 空 mask | `invalid_mask` |
| mask/depth 形状不一致 | `ValueError` |

完整测试：**99 passed**（77 原有 + 20 矿石场景/转换器 + 2 几何）。

## 6. 限制与风险

1. **无真实深度**：所有结果来自合成平面；真实 D435i/Aurora 噪声、空洞、
   反光、粉尘未覆盖。
2. **协方差未校准**：`ddof=1` 样本协方差，不是测量不确定度；没有深度噪声
   模型/NEES 验证。
3. **可见 != 完整**：中心/尺寸/OBB 只描述可见部分，不能当完整矿石几何。
4. **无支撑面/遮挡方向**：未做 table plane、支撑关系或遮挡顺序。
5. **未接入 schema v2**：几何结果目前是模块输出，不写入 `jcheng.scene_state.v1`
   的 `geometry` 字段；S3 再设计 v2。
6. **依赖 core**：复用 `jcheng_perception_core.geometry` 的反投影/变换/四元数，
   包依赖已在 `package.xml` 声明；不复制公式。
7. **无运行时**：不发布 topic、不读设备、不接 `ExecuteMotion`。

## 7. 验收与下一步

本轮验收：

- 几何数学/失败路径测试通过；
- 输出只含 `visible_*`，真实质心/完整尺寸/体积保持 unknown；
- 无硬件/运行时改动。

下一步（S2 真正验证）：

1. 完成 D435i 对齐深度（现有 ADR 0004/M4）。
2. 用静态已知几何（平面/立方体/标定球）测可见中心重复性、深度有效率、
   协方差校准（NEES）。
3. 再上真实矿石录制；阈值由 Board 基于实测 baseline 设定。
4. S3 设计 `jcheng.scene_state.v2` 的 geometry 字段与迁移。

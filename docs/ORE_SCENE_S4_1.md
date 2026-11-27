# S4.1 合成双视角生成器

> 日期：2026-11-27。状态：**Partially verified（仅生成器机制，合成数据）**。
>
> 本轮实现 S4 的离线数据底座：两个已标定视角的 RGB、metric z-depth、实例
> label，以及由构造决定的跨视角真值对。**尚未实现 S4.2 关联匹配器**，因此
> 没有任何 association P/R/F1，也不能声称多视角关联可用。

## 1. 为什么先做生成器

S4 的关联算法需要“同一物理实例在两个视角中分别是哪个 mask”的真值。真实
双摄矿石数据没有现成实例对应标注，公开矿石数据集也基本是单视角。没有可控
真值就调不出门控、也分不清“关联错误”和“分割错误”。因此先做离线生成器：

- 几何、外参、遮挡、实例对应全部由构造决定；
- 可重复、可分层（密度/遮挡/空场景）；
- 不接触真机、DDS、设备或运动路径。

## 2. 场景与相机模型

### 2.1 世界与实例

- 世界帧 `base_link`，z 轴向上，z=0 为支撑面；
- 矿石用椭球代理：中心 `(x, y, c)`，半轴 `(a, b, c)`，仅绕 z 轴 yaw；
- 因中心高度等于垂直半轴，椭球与支撑面在底部相切；
- 实例在半径 0.18 m 的圆盘内均匀采样，允许相互重叠以产生接触/遮挡；
- 每个实例有构造级 `global_id`，不随视角变化。

### 2.2 相机

两个固定合成相机：

| view_id | 位置 `base_link` (m) | 看向 | FOV |
|---|---|---|---|
| `ext` | `(0.52, -0.38, 0.42)` | `(0, 0, 0.026)` | 55° |
| `wrist` | `(0.38, 0.34, 0.36)` | `(0, 0, 0.026)` | 55° |

相机帧遵循 ROS optical 约定：+z 向前、+x 向右、+y 向下。内参由宽度和
FOV 计算，`cx=(w-1)/2`、`cy=(h-1)/2`；`camera_to_base` 为 4×4 齐次变换，
一并写入 manifest。外参是**合成值**，不代表真实标定。

### 2.3 渲染

`render_view` 对每个像素发出世界坐标射线，与每个椭球求交：

- 取最近正交点作为 z-buffer，得到实例 label；
- depth 为相机 z 方向的距离（metric，float32），背景为 0；
- 对每个实例额外统计“无遮挡 silhouette”像素数，用于计算
  `occlusion_ratio = 1 - visible / full_silhouette`；
- RGB 用 Lambert 光照 + 每实例确定性纹理；背景为暗色皮带近似。

实现只依赖 numpy/OpenCV，无 torch、无 GPU、无 ROS。

## 3. 输出格式

```text
<out>/
├── dataset.json          # manifest：相机、生成器、场景、文件 sha256
├── pairs.jsonl           # 每场景的构造真值跨视角对 / singleton
├── images/<scene>_<view>.png
├── masks/<scene>_<view>.png     # uint16 实例 label，0 为背景
├── depth/<scene>_<view>.npy     # float32 z-depth，0 为无效/背景
└── DATASET_CARD.md
```

`dataset.json` 中每个场景包含：

- 两视角的 `image`/`mask`/`depth` 路径与 sha256；
- 每实例的 `global_id`、中心/半轴/yaw、两视角的
  `instance_id`/`mask_index`/`bbox_xyxy`/`visible_area_px`/
  `full_silhouette_px`/`occlusion_ratio`/`centroid_px`；
- `attributes.n_pairs`、`attributes.n_singletons`、`empty`。

`pairs.jsonl` 每行：

```json
{"scene_id": "...", "pairs": [{"global_id": "...", "members": [
  {"view_id": "ext", "instance_id": "..."},
  {"view_id": "wrist", "instance_id": "..."}]}],
 "singletons": [{"view_id": "ext", "instance_id": "...",
                 "global_id": "...", "reason": "not_visible_in_wrist"}]}
```

真值规则：

- 两视角都可见 → 一条 pair；
- 仅一个视角可见 → singleton，原因是 `not_visible_in_<other>`；
- 两视角都不可见 → 不进入 pairs/singletons，但实例仍保留在场景中。

## 4. CLI

```bash
python3 scripts/ore_scene.py generate-dual-view \
  --out /tmp/dual-view \
  --scenes 6 --empty-scenes 1 --seed 20261127 \
  --width 480 --height 360 \
  --min-instances 8 --max-instances 18
```

参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--scenes` | 6 | 非空场景数 |
| `--empty-scenes` | 1 | 空场景数 |
| `--seed` | 20261127 | 确定性种子 |
| `--width/--height` | 480/360 | 图像尺寸，最小 64 |
| `--min/max-instances` | 8/18 | 非空场景实例数范围 |

生成器 API：

```python
from jcheng_perception_scene.synthetic_dual import (
    generate_dual_view_dataset, generate_dual_view_scene,
    render_view, look_at_camera, Camera, Ellipsoid,
)
```

## 5. 测试

新增 5 个测试（S4.1 结束时总计 110；加入 S4.2 后当前 **114 passed**）：

1. `test_dual_view_dataset_ground_truth`：manifest/label/depth/bbox/pair/
   singleton 一致；空场景全零；
2. `test_dual_view_render_occlusion_and_depth`：前球遮挡后球，中心像素
   label/深度正确，后球仍有可见环；
3. `test_dual_view_dataset_is_deterministic`：同参数两次生成的场景记录
   逐项相等（含 sha256）；
4. `test_dual_view_cli`：CLI 生成 manifest 与 pairs 文件；
5. `test_dual_view_rejects_invalid_ranges`：实例数范围、图像尺寸、相机
   重合等非法输入被拒绝。

证据：`evidence/ore_scene_s4_1_20261127/`（4 场景 + 1 空场景，61 对，
2 singleton，全部一致性检查通过）。完整数据集不提交，可用上面的命令重生成。

## 6. 限制

1. **椭球代理**，没有真实矿石的破碎面、纹理、粉尘、水膜、镜面反射。
2. 相机位姿固定且无误差；未模拟真实标定残差、热漂移、臂部遮挡、运动模糊。
3. 深度是理想 z-buffer，没有传感器噪声、空洞、多路径或边缘飞点。
4. 真值来自构造，不是人工关系标注；不能替代真实域评测。
5. 尚无关联匹配器，因此没有 association P/R/F1；真实矿石关联精度
   **Unverified**。
6. 无 DDS/设备/运动/`jcheng-interfaces` 改动。

## 7. 下一步

S4.2 匹配器已实现，见 [`ORE_SCENE_S4_2.md`](ORE_SCENE_S4_2.md)：候选门控 +
互相投影支持 + 匈牙利 + `ambiguous`，在 8 个稠密合成场景（125 对）上
P/R/F1 1.000/0.944/0.971。下一步是 S4.3 融合/序列化与 S4.4 敏感性/消融；
任何运行时/契约暴露都留到 S4.4 证据之后另案评审。

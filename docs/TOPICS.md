# 话题、服务与 QoS

## 1. 输入（厂商相机驱动，只读）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | D435i 彩色；RealSense 驱动默认命名 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | D435i 深度，**必须对齐到彩色** |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | D435i 内参 |
| `/depth_cam/rgb/image_raw` | `sensor_msgs/Image` | Aurora RGB（640×400，~12 Hz） |
| `/depth_cam/depth/image_raw` | `sensor_msgs/Image` | Aurora 深度（640×400，~8 Hz，毫米） |
| `/depth_cam/rgb/camera_info` | `sensor_msgs/CameraInfo` | Aurora RGB 内参 |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `base_link ← camera_optical_calibrated` / `← camera_color_optical_frame` |

上述话题名可在 `config/cameras.yaml` 修改；若相机以压缩话题发布，把
`rgb_compressed` / `depth_compressed` 设为 `true`。

## 2. 上行（Pi 5 → GPU）

### `/perception/uplink/<camera_id>/image_compressed`

`sensor_msgs/CompressedImage`，best effort，depth 1。

- `format: jpeg`
- `header.stamp` = **采集时间**（驱动 stamp；为零时才用到达时间）
- `header.frame_id` = 驱动帧名（仅诊断；几何使用配置的光学帧）

接收端不得改写 stamp。上行按 `max_uplink_fps` 限速（含 15% 抖动容差）；
编码在工作线程，队列满丢最旧。

## 3. 检测（GPU → Pi 5）

### `/perception/detections/<camera_id>`

`jcheng_perception_msgs/Detection2DArray`，best effort，depth 10。

```text
std_msgs/Header header        # stamp = 上行采集时间，绝不改成推理时间
string camera_id
string model_id               # 例如 ext_yolov8n.pt:1763...（含 mtime）
uint64 frame_seq              # GPU 侧计数，仅诊断
uint32 image_width/height     # 原图尺寸
float32 inference_ms
Detection2D[] detections
```

`Detection2D`：

| 字段 | 语义 |
|---|---|
| `label` | 类别名（`labels` 覆盖或模型 names） |
| `class_id` | 类别索引 |
| `score` | 置信度 |
| `bbox_xyxy[4]` | **原图**像素坐标，已从 letterbox 还原 |
| `keypoints[]` | 扁平 `x,y,confidence` 三元组；可为空 |
| `track_hint` | 预留，当前为 0 |

### 影子检测

`shadow_gpu.launch.py` 启动的第二个 GPU 节点读取同一上行，但发布到：

- `/perception/detections_shadow/<camera_id>`
- `/perception/debug_shadow/<camera_id>`
- `/perception/gpu_status_shadow`

edge 不订阅这些话题，因此影子模型不会影响融合。生产/影子节点不能发布同一个
检测话题。

## 4. 单相机 3D 观测（Pi 5 内部/调试）

### `/perception/observations/<camera_id>`

`jcheng_perception_msgs/TargetObservation`，best effort，depth 10。

- `header.stamp` = **深度帧时间戳**（3D 测量的时间）
- `header.frame_id` = `target_frame`（默认 `base_link`）
- `role`：`ROLE_EXTERNAL=1` / `ROLE_WRIST=2`
- `pose`：`PoseWithCovariance`；平移块来自深度/像素噪声，`rz` 来自方向估计，
  `rx/ry` 标记为未观测（1.0）
- `depth_m`、`valid_depth_fraction`、`occlusion_ratio`
- `view_weight`：本次融合中的归一化权重（未匹配到轨迹时为 0）
- `camera_frame`：配置的光学帧

该话题是调试/证据通道，规划器应只消费 `/perception/targets`。

## 5. 融合目标（Pi 5 → 规划器）

### `/perception/targets`

`jcheng_perception_msgs/FusedTargetArray`，best effort，depth 10，30 Hz。

`FusedTarget`：

| 字段 | 语义 |
|---|---|
| `target_id` | `label-NNN`，生命周期内稳定 |
| `label` / `class_id` | 目标类别 |
| `pose` | `base_link` 下的位置 + 方向；6×6 行主序协方差 |
| `confidence` | 加权检测置信度 × 时间衰减 |
| `phase` | `SEARCH/COARSE/ACQUIRE/FINE/DROP_RECHECK` |
| `status` | `TENTATIVE/CONFIRMED/COASTING/LOST` |
| `weight_external` / `weight_wrist` | 本次融合各视图权重，和 ≤ 1 |
| `depth_external_m` / `depth_wrist_m` | 各视图最近深度 |
| `occluded` | 融合遮挡比例 ≥ 0.5 |
| `recheck_required` | `DROP_RECHECK` 期间外机位参与 |
| `source_cameras` | 本次融合贡献的相机 |
| `age_s` | 轨迹总年龄 |

- 数组按 `confidence` 降序。
- 无 confirmed 目标时发布空数组；**不要**把空数组解释为“保持当前位姿”。
- 规划器必须检查 `header.stamp` 与自身时钟的差值（建议 > 200 ms 视为过期）。

## 6. 状态

### `/perception/status`（edge，1 Hz）

`PerceptionStatus`：每相机 FPS、累计帧/检测/匹配/丢弃数、匹配率、平均检测
延迟、平均深度匹配年龄、相位、GPU 存活、轨迹数、备注（depth/TF 失败计数）。

### `/perception/gpu_status`（GPU，2 s）

`GpuStatus`：每相机 FPS、平均推理耗时、接收/丢弃/失败帧数、模型 ID（含 mtime）。

## 7. 服务

| 服务 | 类型 | 作用 |
|---|---|---|
| `/jcheng_perception_edge/drop_recheck` | `std_srvs/Trigger` | 强制 `DROP_RECHECK` 3 s（可配），外机位主导 |
| `/jcheng_perception_edge/reset` | `std_srvs/Trigger` | 清空轨迹与待处理检测，保留环形缓冲 |

示例：

```bash
ros2 service call /jcheng_perception_edge/drop_recheck std_srvs/srv/Trigger
ros2 service call /jcheng_perception_edge/reset std_srvs/srv/Trigger
```

## 8. QoS 注意事项

- `ros2 topic echo` 默认 reliable，**读不到 best-effort 话题**。必须加：

  ```bash
  ros2 topic echo --qos-reliability best_effort /perception/targets
  ros2 topic hz --qos-reliability best_effort /perception/targets
  ```

- `CameraInfo` 为 reliable + transient local；迟到订阅者也能收到最后一帧。
- 改变 QoS 前先读 [ARCHITECTURE.md](ARCHITECTURE.md) 的“为什么 best effort”：
  reliable 周期状态流会被慢/失联订阅者反压，进而拖住执行器。

## 9. 临时 IDL

`jcheng_perception_msgs` 目前是**仓库内临时契约**，尚未进入
`JCHENG-Robotics/jcheng-interfaces`。M2 契约审计后再提升；在此之前字段变更
不承诺兼容。规划器集成时应固定本仓库的提交或版本标签。

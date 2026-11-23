# 时间戳与 TF 契约

## 1. 时钟权威

**Pi 5 是唯一时间权威。** 采集时间由相机驱动写入 `header.stamp`；edge 的
`CompressedImage`、GPU 的 `Detection2DArray`、edge 的 `TargetObservation` 都
沿用这个时间，不做任何“推理时刻”改写。

```text
Pi5 driver stamp t0
  → CompressedImage.header.stamp = t0
  → Detection2DArray.header.stamp = t0        (GPU: 原样)
  → 查 depth ring @ t0 → 得到 depth stamp td
  → 查 pose ring @ td  → 得到 T_base_camera(td)
```

为什么只信 Pi 5 的时钟：GPU 与 Pi 5 的系统时钟即使有几十毫秒偏差，也不影响
匹配，因为 GPU 不做时间戳运算，只做“携带”。跨机日志对齐才需要 NTP/chrony。

## 2. 时间戳来源与降级

| 情形 | 行为 |
|---|---|
| 驱动 stamp > 0 | 直接使用 |
| 驱动 stamp = 0 | 用 edge 到达时间；该帧的“采集时间”精度下降，会在日志中体现为延迟偏小；建议修复驱动而不是长期依赖 |
| 时钟跳变/回拨 | 环形缓冲按排序插入，不崩溃；但年龄剪枝会异常。应使用 chrony 平滑时钟，跳变后重启节点 |

## 3. 匹配窗口（默认值，config 可改）

| 相机 | 帧匹配 | 深度匹配 | TF 匹配 | 理由 |
|---|---|---|---|---|
| D435i（external） | 50 ms | **200 ms** | 60 ms | 彩色 15 Hz；彩色+对齐深度实测可能只有 4–8 Hz，外机位静止，旧深度仍空间有效 |
| Aurora（wrist） | 80 ms | **150 ms** | 80 ms | RGB ~12 Hz，深度 ~8 Hz；USB2 带宽限制 |
| 合成演示 | 60 ms | 80 ms | 80 ms | 10 Hz 同源，无速率差 |

- 帧匹配：用检测 stamp 找最近的 `FrameMeta`（尺寸、K、D、frame_id）。
- 深度匹配：用检测 stamp 找最近的深度帧。
- TF 匹配：**用深度帧时间戳**，不是 RGB 时间戳（见下节）。
- 任一步失败 → 丢弃该检测，计入 `frame_match_failures` / `depth_match_failures`
  / `tf_match_failures`；不会退回“最新帧”。

## 4. 为什么 TF 用深度时间戳

深度像素 `(u,v,d)` 定义的是**深度帧拍摄瞬间**相机坐标系下的 3D 点：

```text
p_camera(td) = [ (u-cx)d/fx, (v-cy)d/fy, d ]
p_base       = T_base_camera(td) · p_camera(td)
```

RGB 框只负责“选哪个像素”；真正的 3D 坐标属于深度帧。若腕相机在 RGB 与深度
之间运动（8–12 Hz 下可达几十毫秒），用 RGB 时刻的 TF 会把相机位移引入深度点。
因此流水线在 `synchronize()` 中用 `depth_stamp` 查 TF，并把这个时间写入观测。

残余误差：RGB 框与深度帧之间的相机运动会让像素选择略有偏差；容差窗口和
深度/彩色对齐要求把这项控制在毫米级。更严格的做法是硬件同步，见
[DEPLOYMENT.md](DEPLOYMENT.md)。

## 5. TF 维护与插值

- edge 用 `tf2_ros.Buffer(cache_time=10 s)` + `TransformListener`。
- 定时器每 `pose_sample_hz`（默认 30 Hz）对每相机做一次
  `lookup_transform(target_frame, optical_frame, Time())`，把 4×4 矩阵写入
  `PoseRingBuffer`。
- 静态外机位只每 2 s 采样一次（0.5 Hz）：实测 100 Hz 双相机采样会占满一个
  edge 核。腕相机 30 Hz。
- 查询时：先按精确 stamp 找；否则在相邻两帧之间线性插值平移、slerp 插值旋转；
  间隔超过 `max_gap` 则拒绝。
- 若 pose ring 无可用样本，外机位可回退到 `config/frames.yaml` 的
  `T_base_front`（2026-11-20 实测）。**腕相机没有静态回退**：缺失 TF 就丢观测，
  这是刻意 fail-closed。

## 6. 帧名契约（复用 `jcheng-manipulation`）

| 相机 | 光学帧 | 说明 |
|---|---|---|
| D435i | `camera_color_optical_frame` | RealSense 驱动；若走 UVC/`ext_cam_optical`，改配置 |
| Aurora | `camera_optical_calibrated` | 手眼标定的 OpenCV 光学帧；**不要**用 `rgb_camera_link` 或 `depth_cam_color_optical_frame` |

`rgb_camera_link` 是驱动 body 帧；`depth_cam_color_optical_frame` 在实机上与
`rgb_camera_link` 恒等，并不是光学帧。详见
`jcheng-manipulation/docs/hardware/camera_frames.md`。

## 7. 深度/彩色对齐要求

- D435i：必须启用 `align_depth_to_color`，深度与彩色同分辨率、同光学帧。
- Aurora：驱动输出的深度与 RGB 已按 `rgb_camera_link` 对齐（TF 视为重合）；
  使用标定光学帧做反投影。
- 若深度/彩色分辨率不一致，`synchronize()` 直接丢弃并计
  `depth_match_failures`。
- 内参：优先使用 `CameraInfo`；消息尺寸不匹配或尚未到达时使用
  `cameras.yaml` 的 `intrinsics` 回退；两者都没有则跳过该帧。

## 8. 可观测性

`/perception/status` 提供：

- `mean_detection_latency_ms`：检测回调时刻 − 采集时刻（含网络、GPU、队列）。
- `mean_depth_match_age_ms`：深度帧与检测 stamp 的差。
- `detections_matched` / `detections_dropped` / `match_success_rate`。
- `notes`：每相机 `depth_failures` / `tf_failures` 计数。

排查顺序：

1. `ros2 topic hz --qos-reliability best_effort /perception/uplink/<cam>/image_compressed`
2. `... /perception/detections/<cam>`（应≈上行率）
3. `ros2 run tf2_ros tf2_echo base_link camera_optical_calibrated`
4. `/perception/status` 的匹配率与失败计数
5. 若延迟高：检查 GPU `GpuStatus.frames_dropped`、队列、模型推理耗时

## 9. 跨机时钟建议

- 机器人 eth0 与笔记本有线直连，双方启用 chrony（或 PTP，若交换机支持）。
- 时间戳匹配不依赖跨机同步；同步只用于把两台机器的日志放在同一时间轴。
- 若发现 Pi 5 时钟被 NTP 大步调整，重启 edge 节点，避免环形缓冲年龄异常。

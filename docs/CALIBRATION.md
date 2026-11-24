# 标定与坐标系

本仓库**不重新定义**机器人标定，直接复用 `jcheng-manipulation` 的实测结果。
相机重装、分辨率变化或镜头更换后，必须重新标定并更新配置。

## 1. 复用的实测结果

### 1.1 腕部 Aurora 930

- 光学帧：`camera_optical_measured`（OpenCV 光学：x 右、y 下、z 前）。
- 发布方式：本仓库 `wrist_tf` 读取总线实测关节，做 `base_link→link4` FK，
  再组合下面的手眼，发布为 `base_link` 的动态子帧（10 Hz）。
- 为什么不用静态 `camera_optical_calibrated`：厂商 `/joint_states` 是命令缓存，
  与总线实测可差数百脉冲；静态帧的父链会错。2026-11-24 实测命令缓存
  joint1=+0.838、joint2–5=0，而总线 joint1≈-0.013、joint2≈+0.406、
  joint3≈-1.382、joint4≈-1.629。
- 手眼（TSAI，14 姿态，使用总线实测关节）：

```text
T_link4_camera
translation = [-0.0348, -0.0091, 0.0586] m
RPY         = [0.951, -0.058, -89.789]°
board scatter 2.94 mm / 0.39°
held-out      3.29 mm / 0.43°
```

- 驱动内参（640×400，plumb_bob）：

```text
K = [418.0542, 0, 308.7825; 0, 419.1000, 187.1342; 0, 0, 1]
D = [0.00960977, -0.05017150, 0.00056402, -0.00070902, 0.03192810]
```

- 注意：`rgb_camera_link` 是 **body** 帧；`depth_cam_color_optical_frame`
  在实机上与它恒等，不是光学帧。反投影必须用标定光学帧
  `camera_optical_measured`。

### 1.2 车头 D435i

> **当前部署注意**：实机目前是 `ext_cam_node`（UVC 彩色，`ext_cam_optical`）+
> 仅深度 `realsense2_camera`（`camera_depth_optical_frame`）的分离模式。彩色与
> 深度未对齐、非同源时间戳，不能在同一像素上做度量反投影。本仓库默认配置假设
> 单一 RealSense 节点发布彩色 + `aligned_depth_to_color`（`camera_color_optical_frame`）。
> 切换前见 [ADR 0004](adr/0004-camera-device-ownership.md)。

- 安装：车头固定，`base_link ← ext_cam_optical` 为常量。
- 2026-11-20 标定：

```text
translation = [0.13930, 0.02704, 0.03675] m
pitch below horizontal = 32.62°
yaw = -1.29°
table z(base_link) = -0.0825 m
camera height above table = 0.1193 m
```

- UVC 彩色内参（1280×720，RMS 0.51 px）：

```text
fx = fy = 892.1014568
cx = 648.0281251
cy = 380.1575036
D = [0.14971375, -0.44610982, 6.162e-06, -0.00012207, 0.34121030]
```

- 若使用 RealSense 驱动，`CameraInfo` 的内参会覆盖配置回退；两者应在
  ~1 px 内一致。不一致时以驱动 `CameraInfo` 为准并重新核对标定。
- 深度必须 **align to color**；否则像素与深度不对应。

### 1.3 桌面高度

`base_link` 原点不在桌面。合成场景与点云去桌面使用
`table_z = -0.0825 m`（`config/cameras_synthetic.yaml` 的合成场景常量、
`fusion.table_clearance_m` 用于去桌面）。实机若桌面高度变化，需要更新：

- `scripts/simulate_pipeline.py` / 合成场景的 `TABLE_Z`；
- `fusion.yaml` 的 `table_clearance_m` 只是“高于拟合桌面的阈值”，不是绝对高度；
  绝对桌面由深度点云拟合得到，无需配置。

## 2. 配置文件位置

| 内容 | 文件 |
|---|---|
| 相机话题、光学帧、内参回退、深度噪声/量程 | `config/cameras.yaml` |
| 腕部总线实测 TF、手眼矩阵、伺服映射 | `config/wrist_tf.yaml` |
| 外机位静态回退 `T_base_front` | `config/frames.yaml` |
| 合成场景内参/话题 | `config/cameras_synthetic.yaml` |
| 融合/方向/合并参数 | `config/fusion.yaml` |

腕相机**没有**静态外参回退：`T_base_link ← camera_optical_measured` 必须由
`wrist_tf` 实时发布。缺失 TF 时观测被丢弃。启动 `wrist_tf` 前需 source 厂商
工作区（`ros_robot_controller_msgs`）。

## 3. 重新标定的触发条件

- 相机支架松动、碰撞、重新安装；
- 更换相机/镜头/分辨率；
- 深度-彩色对齐参数变化；
- 固定目标在 base 下的残差持续超过误差预算；
- 外机位静态标定后底盘/云台结构改动。

腕相机手眼流程见 `jcheng-manipulation/docs/phase2/CALIBRATION.md`；外机位
流程见 `evidence/calibration/20261120/` 与
`tools/phase2/front_camera_extrinsic.py`。**不要**修改 `jcheng-manipulation`
的标定证据；本仓库只记录引用与配置值。

## 4. 验证流程

1. 桌面放一个固定、有纹理/深度的目标（方块或标定板）。
2. 启动 `wrist_tf`，确认 `ros2 run tf2_ros tf2_echo base_link
   camera_optical_measured` 的 z 轴朝前下方；再启动 edge + GPU，静止采集 ≥ 5 s。
3. 记录 `/perception/observations/d435i_ext` 与 `.../aurora_wrist` 的位置：
   - 两视图在 base 下的差应接近标定误差量级；
   - 目标在桌面时 z 应接近桌面 + 物体半高。
4. 移动目标，检查 `/perception/targets`：
   - 轨迹 ID 不跳变；
   - 遮挡外机位 1 s，腕相机应继续输出；
   - `drop_recheck` 后权重变为 external 1.0。
5. 记录证据（JSON/日志/截图）到 `evidence/`，不要只写“看起来正常”。

## 5. 误差来源

| 来源 | 量级 | 影响 |
|---|---|---|
| 腕相机手眼 | 2.9–3.3 mm / 0.4° | 近距离抓取 |
| 腕部关节读数 | 10 Hz、单次 5 关节读取 34 ms；总线 ±1 pulse | 运动中 TF 的滞后 |
| 腕部深度/彩色配准 | IR-RGB 约 9.9 mm 未建模 | 近距离像素选择 |
| 外机位静态外参 | 棋盘 0.8° 平面、深度拟合 30–36° 俯仰 | 粗定位 |
| 深度噪声 | z² 相关；见 `q/floor` | 融合权重 |
| 深度/彩色时间差 | 8–12 Hz 下的相机运动 | 像素选择偏差 |
| 关节读数/臂形变 | 现有基线预算 | 腕相机 TF |
| 目标方向歧义 | ±90°/180° | 抓取 yaw |

这些值不是本仓库的新测量；实机部署后应重新做误差预算。

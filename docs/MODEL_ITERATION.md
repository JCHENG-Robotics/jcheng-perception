# 模型迭代与调试

## 1. 目标

- 换模型/换权重**不重启机器人节点**，不影响采集、TF、融合。
- 两路相机可以各自使用不同模型（外机位检测、腕部关键点）。
- 每次检测都携带 `model_id`（权重文件 mtime），日志/话题可追溯。
- 先用影子模型对比，再切换生产话题。

## 2. 模型放置与配置

权重不提交到仓库。把模型放到 GPU 主机任意路径：

```yaml
# config/gpu.yaml
cameras:
  d435i_ext:
    input_topic: /perception/uplink/d435i_ext/image_compressed
    detections_topic: /perception/detections/d435i_ext
    hot_reload: true
    model:
      backend: ultralytics        # 或 mock
      path: /opt/jcheng/models/ext_yolov8n.pt
      task: detect
      imgsz: 640
      conf: 0.35
      iou: 0.45
      device: "0"
      half: true
      labels: []                  # 可选：按 class_id 覆盖名称
      keypoints: false
  aurora_wrist:
    ...
    model:
      path: /opt/jcheng/models/wrist_grasp_pose.pt
      task: pose                  # 关键点模型；detect-only 模型改为 detect
      keypoints: true
```

支持的路径：Ultralytics `.pt`、`.onnx`、`*_openvino_model/` 目录。厂商
`yolov8_detect` 的 OpenVINO 导出可直接复用。

> **2026-11-25 权重状态**：本机 `/opt/jcheng/models/` 只有
> `fallback_yolov8n_coco.pt`（stock COCO，80 类，仅用于基础设施验证）；
> 生产用的 `ext_yolov8n.pt` 与 `wrist_grasp_pose.pt` **仍缺失**。已检查
> 本机、厂商源码、SD 卡/备份、工作站 `ti5robot`（192.168.5.31）以及机器人
> 的 Docker overlay：只有厂商 garbage/mask/yolov8s 模型和一份 25 张的
> orange/Paizi 演示集，**没有本项目权重，也没有可训练的标注数据集**。当前
> 已验证的路线是 `config/gpu_tag.yaml` 的 AprilTag 参考点模式；没有生产
> 权重时用 `config/gpu_fallback.yaml`，它不做任何任务精度声明。检查命令：
> `python3 scripts/check_models.py --config <config> --load`。

### 后端选择

| `backend` | 运行时 | 支持任务 | 适用场景 |
|---|---|---|---|
| `ultralytics` | Ultralytics + torch | detect / pose / obb | GPU 主机首选；关键点模型必须用它 |
| `onnxruntime` | ONNX Runtime | detect | CPU/ORT-GPU；厂商 `yolov8s.onnx` 可直接用 |
| `openvino` | OpenVINO Runtime | detect | Intel CPU/GPU；厂商 `.xml/.bin` IR |
| `opencv` | OpenCV DNN | detect | 较老 YOLO ONNX；**OpenCV 4.6 不能解析 YOLOv8 DFL** |
| `mock` | 无 | 颜色阈值 | CI、无模型演示 |

当前本机 venv：`~/.venvs/jcheng-perception-gpu`，已装 ONNX Runtime 1.32.0、
OpenVINO 2026.4.1、Ultralytics 8.4.216、torch 2.9.1+cu128、
torchvision 0.24.1+cu128；RTX 3060 可用，`yolov8s.pt` 稳态 9.0 ms（FP32）/ 7.4 ms（FP16）。
CPU 版 torch 2.6.0+cpu 曾用于无 GPU 验证，已被 CUDA 版替换。

## 3. 运行前置检查

在 GPU 主机上，先用 ROS 无关的烟测确认后端与模型。没有额外运行时可用 mock：

```bash
python3 scripts/check_gpu_backend.py --backend mock
```

安装可选运行时（`~/.venvs/jcheng-perception-gpu`，带 system site-packages，
不污染 ROS Python）：

```bash
# ONNX Runtime CPU（本机已验证）
RUNTIME=onnxruntime ./scripts/setup_gpu_venv.sh
# OpenVINO CPU/Intel GPU（本机已验证）
RUNTIME=openvino ./scripts/setup_gpu_venv.sh
# Ultralytics + CPU torch（无需 CUDA）
RUNTIME=ultralytics-cpu ./scripts/setup_gpu_venv.sh
# Ultralytics + CUDA torch（RTX 3060；固定 2.9.1/cu128 组合）
RUNTIME=ultralytics-cuda ./scripts/setup_gpu_venv.sh
# CUDA Ultralytics（GPU 主机；下载量大）
RUNTIME=ultralytics ./scripts/setup_gpu_venv.sh
```

真实模型单图烟测：

```bash
# ONNX Runtime：厂商 yolov8s.onnx，CPU，4 个框 / 197.5 ms（本机实测）
~/.venvs/jcheng-perception-gpu/bin/python scripts/check_gpu_backend.py \
  --backend onnxruntime \
  --model /path/to/yolov8s.onnx \
  --image /path/to/sample.jpg

# OpenVINO：厂商 yolov8s.xml，CPU，4 个框 / 126.7 ms（本机实测）
~/.venvs/jcheng-perception-gpu/bin/python scripts/check_gpu_backend.py \
  --backend openvino \
  --model /path/to/yolov8s.xml \
  --image /path/to/sample.jpg \
  --device CPU

# 关键点模型（需要 Ultralytics）
python3 scripts/check_gpu_backend.py \
  --backend ultralytics \
  --model /opt/jcheng/models/wrist_grasp_pose.pt \
  --task pose --image /data/wrist_sample.jpg
```

输出包含 `model_id`、加载/推理耗时、检测框与关键点。缺少运行时或模型路径时
脚本以退出码 2 返回并给出明确原因；它不会自动安装依赖。

部署前预检（2026-11-25 新增）：

```bash
# 1) 权重：列出缺失文件并实际加载
python3 scripts/check_models.py \
  --config ros2_ws/src/jcheng_perception_gpu/config/gpu.yaml --load
# 没有生产权重时，仅做基础设施验证：
python3 scripts/check_models.py \
  --config ros2_ws/src/jcheng_perception_gpu/config/gpu_fallback.yaml --load

# 2) 相机速率（Humble + 有线 CycloneDDS 容器内运行）
python3 scripts/check_camera_rates.py --duration 10   # 阈值 RGB 6 / depth 5 Hz
# 低帧率：sudo systemctl restart armpi-vision（约 2.5 分钟就绪）
# ingress 看门狗会自动重启进程；若 unit 到达启动上限再 reset-failed
```

ROS 节点用 venv 解释器启动（console script 的 shebang 仍指向系统 Python，
`python_executable` 会把脚本作为参数交给 venv Python）：

```bash
ros2 launch jcheng_perception_gpu laptop_gpu.launch.py \
  gpu_config:=/abs/path/gpu_onnx.yaml \
  python_executable:=$HOME/.venvs/jcheng-perception-gpu/bin/python
```

`config/gpu_onnx.example.yaml` 与 `config/gpu_openvino.example.yaml` 是两种
CPU 后端的模板。

## 4. 热加载

- worker 每秒检查模型路径 mtime（文件或目录内最新文件）。
- 变化后在下一帧前重新构造 `YOLO`；加载失败保留旧模型并打印错误。
- `model_id` 变为 `<basename>:<mtime_ns>`，出现在 `Detection2DArray.model_id`
  和 `GpuStatus.model_ids`。
- 首次启动加载失败不会崩溃：worker 每秒重试，`gpu_status.frames_failed`
  会体现推理失败。

## 5. 标准迭代流程

1. **采集**：在机器人上录制短片段（见第 6 节）。
2. **训练/导出**：
   ```bash
   yolo export model=runs/pose/train/weights/best.pt format=openvino imgsz=416
   ```
3. **离线评估**：mAP / 关键点 PCK、单帧推理耗时、漏检/误检；用
   `scripts/simulate_pipeline.py` 回归融合逻辑（不依赖模型）。
4. **影子运行**：复制 `config/gpu_shadow.example.yaml` 为本地文件，改权重路径；
   它已经把检测/调试话题指向 `*_shadow`。启动第二个 GPU 节点：
   ```bash
   ros2 launch jcheng_perception_gpu shadow_gpu.launch.py \
     gpu_config:=/abs/gpu_shadow.yaml
   ```
   edge 只订阅生产 `/perception/detections/*`，影子结果不影响融合。影子节点的
   健康状态在 `/perception/gpu_status_shadow`（可用 `status_topic` 覆盖）。
   用 bag 或订阅脚本对比生产/影子检测流。
5. **切换**：把生产 `gpu.yaml` 的 `path` 指向新权重（或直接替换文件触发
   热加载）；观察 `/perception/status` 的匹配率、延迟、轨迹数。
6. **回滚**：保留旧权重与旧配置；mtime 变化会立即切回。

**不要**同时让两个 GPU 节点发布同一个 `detections_topic`：边缘会把两个模型的
框混在一起。

## 6. 数据采集

短窗口（10–30 s）足够调试：

```bash
# Pi 5：原始相机 + TF + 内参
ros2 bag record -o /data/perception_$(date +%s) \
  /depth_cam/rgb/image_raw /depth_cam/depth/image_raw \
  /depth_cam/rgb/camera_info /camera/color/image_raw \
  /camera/aligned_depth_to_color/image_raw /camera/color/camera_info \
  /tf /tf_static

# 或只录上行 + 检测 + 融合（体积小，用于延迟/关联分析）
ros2 bag record -o /data/perception_meta_$(date +%s) \
  /perception/uplink/*/image_compressed \
  /perception/detections/* /perception/targets /perception/status
```

回放：先播放原始相机 bag，再启动 edge + GPU；检测会按 bag 时间戳重新走
时间匹配。用 `ros2 bag play --clock` 时不要启用 `use_sim_time`，除非所有节点
统一；本仓库节点默认使用系统时钟。更简单的做法是回放上行 bag 到 GPU，
把检测结果录下来离线分析。

## 7. 调试可视化

- GPU：`debug_publish:=true` 发布 `/perception/debug/<cam>`（JPEG 叠加框/键点），
  默认 1 s 一张，避免占用 GPU。
- Edge：`timing_log:=true` 每秒打印回调耗时与 pending；`/perception/observations/<cam>`
  可看单相机 3D 观测、权重、遮挡。
- 融合：`/perception/targets` 的 `weight_external/wrist`、`phase`、`status`、
  `source_cameras` 是调参的主要信号。
- RViz/Foxglove：订阅 `FusedTargetArray` 需要自定义消息包；也可写一个小节点
  转成 `MarkerArray`。

## 8. 评估指标

| 维度 | 指标 | 目标（初始） |
|---|---|---|
| 检测 | mAP50 / 关键点 PCK | 按任务设定 |
| 延迟 | `GpuStatus.inference_ms` p95 | < 40 ms（GPU） |
| 端到端 | `PerceptionStatus.mean_detection_latency_ms` p95 | < 120 ms（有线） |
| 匹配 | `match_success_rate` | > 0.95 |
| 融合 | 固定目标 3D RMSE / 移动目标轨迹 RMSE | 实机标定后设定 |
| 稳定 | 30 min 轨迹 ID 切换次数 | 目标级 0 |
| 降级 | GPU 断开后 COASTING→LOST 时间 | ≈ 0.6–1.8 s |

## 9. 关键点模型约定

- Ultralytics pose 输出 `keypoints.xy` + `conf`；本仓库扁平化为
  `[x,y,conf, ...]`，顺序与模型 keypoint 定义一致。
- 若要用力点轴估计目标 yaw，在 `cameras.yaml` 配置：
  ```yaml
  keypoint_axis_pairs: [[0, 1]]
  ```
  两个键点必须稳定可见；否则保留点云 PCA 方向。
- `keypoints: true` 对 detect-only 模型无害（没有 keypoints 字段就返回空），
  但 `task` 必须与模型类型匹配。

## 10. 不能做的事

- 不要把权重、数据集、厂商 SDK 提交到本仓库。
- 不要在 Pi 5 edge 上加载检测器；静态测试会失败。
- 不要让模型节点发布运动命令或写 `/tf`。
- 不要把“影子模型”接到生产检测话题。

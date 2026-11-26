# 数据集与训练（外部检测 + 腕部 pose）

> 状态：**采集/校验/划分/训练/导出工具链已就绪（2026-11-25）**。类别和腕部
> 关键点约定是**提案**，正式标注前需要确认。数据集和权重都不提交到本仓库。

## 1. 两个模型

| 模型 | 任务 | 默认骨干 | 输入 | 输出 |
|---|---|---|---|---|
| `ext_yolov8n.pt` | detect | `yolov8n.pt` | 640 | 5 类 bbox |
| `wrist_grasp_pose.pt` | pose | `yolov8n-pose.pt` | 416 | 5 类 bbox + 2 keypoints |

外部检测负责粗定位/落点复检；腕部 pose 负责抓取点和闭合方向。两者使用同一
组类别名，融合的 `association_label_agnostic=false` 才能把两视角关联起来。

## 2. 类别（提案）

按 `docs/phase2/OBJECT_GRASP_CATALOG.md` 的对象类型：

```text
0 cube          39.4 mm 立方体
1 blue_block    蓝色方块（lying / upright 是形态，不是类别）
2 red_cylinder  Ø28.7 mm 圆柱
3 vlt_carton    黄色 VLT 纸盒
4 snack_sachet  膨化零食袋
```

类别 id 的顺序就是 `data.yaml` 里 `names` 的顺序，标注后不要改；如果确实需要
按形态分类（`block_lying`/`block_upright`），必须同时修改边缘的标签消费逻辑
并重新评估关联。

## 3. 腕部关键点（提案）

`kpt_shape: [2, 3]`，两个关键点是**夹爪两个接触点**（闭合方向的两端）：

- 0 = 图像左侧接触点，1 = 图像右侧接触点（标注时按图像从左到右排序）。
- 可见性：2 = 清晰可见，1 = 被遮挡但可推断，0 = 不可见。
- 边缘 `cameras.yaml` 打开 `keypoint_axis_pairs: [[0, 1]]` 后，用 0→1 轴估计
  目标 yaw；轴方向有 180° 歧义，由抓取阶段处理。
- 两个关键点必须落在物体表面/接触位置，不要标在背景或夹爪上。

## 4. 采集

数据根目录放在仓库外，默认 `~/jcheng/perception_datasets`。采集脚本只订阅
上行，不发布、不写 `/tf`、不动机械臂：

```bash
docker run --rm --network host \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///tmp/cyclonedds-wired.xml \
  -e ROS2CLI_NO_DAEMON=1 -e HOME=/tmp \
  -v "$HOME/jcheng/jcheng-perception:/repo:ro" \
  -v "$HOME/jcheng/perception_datasets:/data" \
  -v "$HOME/jcheng/humble_ws/cyclonedds-wired.xml:/tmp/cyclonedds-wired.xml:ro" \
  jcheng-perception-gpu:latest bash -lc \
  'source /opt/ros/humble/setup.bash; python3 /repo/scripts/capture_dataset.py \
     --out /data/raw --session cube_01 --duration 20 \
     --notes "cube, view pose, lamp on"'
```

输出：`raw/<session>/<camera>/<stamp_ns>.jpg` + `manifest.csv` + `session.yaml`。

采集协议：

- 每次 10–30 s；同一对象在视野内**不同位置、朝向、遮挡、光照**下重复。
- 外部检测至少 20–40 个 session；腕部在每个 view / pre-grasp / grasp 姿态下
  各拍若干 session；每个模型 300–500 张起步，包含空场景/背景 session。
- 腕部视角需要操作员把机械臂摆到目标姿态；这属于运动操作，按现场规则单独授权。
- 采集前先跑 `scripts/check_camera_rates.py`，确认 Aurora 深度没有退化。

## 5. 标注

标签是 YOLO txt，放在同名 `.jpg` 旁边；空 txt 表示背景/无目标。

```text
# detect: class cx cy w h（归一化）
0 0.512 0.480 0.140 0.110

# pose: class cx cy w h kp0x kp0y v0 kp1x kp1y v1
1 0.512 0.480 0.140 0.110 0.470 0.480 2 0.554 0.480 2
```

可用 Label Studio 或 CVAT 标注后导出 YOLO；若导出的是 JSON/XML，先转成上面的
txt 并放回同名 jpg 旁。标完先校验（`--flat` 表示标签与图片同目录）：

```bash
python3 scripts/check_dataset.py --images ~/jcheng/perception_datasets/raw/<session>/d435i_ext \
  --flat --task detect
python3 scripts/check_dataset.py --images ~/jcheng/perception_datasets/raw/<session>/aurora_wrist \
  --flat --task pose --kpt-shape 2 3
```

## 6. 划分、训练、导出

```bash
# 按 session 划分（20% 整段做 val，避免同场景泄漏）
python3 scripts/split_dataset.py --raw ~/jcheng/perception_datasets/raw \
  --camera d435i_ext --out ~/jcheng/perception_datasets/ext \
  --task detect --names cube,blue_block,red_cylinder,vlt_carton,snack_sachet

python3 scripts/check_dataset.py --root ~/jcheng/perception_datasets/ext \
  --task detect --data ~/jcheng/perception_datasets/ext/data.yaml

# 外部检测（RTX 3060；6 GB 显存下 batch 16 / 640 可用）
~/.venvs/jcheng-perception-gpu/bin/python scripts/train_yolo.py \
  --task detect --data ~/jcheng/perception_datasets/ext/data.yaml \
  --epochs 100 --batch 16 --imgsz 640 --install

# 腕部 pose（yolov8n-pose，416）
python3 scripts/split_dataset.py --raw ~/jcheng/perception_datasets/raw \
  --camera aurora_wrist --out ~/jcheng/perception_datasets/wrist_pose \
  --task pose --names cube,blue_block,red_cylinder,vlt_carton,snack_sachet
~/.venvs/jcheng-perception-gpu/bin/python scripts/train_yolo.py \
  --task pose --data ~/jcheng/perception_datasets/wrist_pose/data.yaml \
  --epochs 100 --batch 16 --imgsz 416 --install

# 可选：导出 OpenVINO/ONNX
~/.venvs/jcheng-perception-gpu/bin/python scripts/export_model.py \
  --weights /opt/jcheng/models/ext_yolov8n.pt --format openvino --imgsz 640
```

`--install` 会把 `best.pt` 复制到 `/opt/jcheng/models/{ext_yolov8n.pt,
wrist_grasp_pose.pt}`。仓库永远不保存权重和数据集。

> 训练保存时若报 `No module named 'polars'`，给 venv 补上：
> `uv pip install --python ~/.venvs/jcheng-perception-gpu/bin/python polars`
> （`RUNTIME=ultralytics-cuda ./scripts/setup_gpu_venv.sh` 已包含）。

## 7. 验证与切换

```bash
python3 scripts/check_models.py --config \
  ros2_ws/src/jcheng_perception_gpu/config/gpu.yaml --load
```

启动 GPU 节点后看 `/perception/gpu_status` 的 `model_ids`/`inference_ms` 和
`/perception/status` 的 `gpu_alive`、`active_tracks`；腕部关键点稳定后，在
`cameras.yaml` 打开 `keypoint_axis_pairs: [[0, 1]]`。先用影子节点
（`shadow_gpu.launch.py`）对比，再把生产 `gpu.yaml` 指到新权重。

## 8. 不能做

- 不把数据集、权重、厂商 SDK 提交到仓库。
- 不把同一 session 的帧同时放进 train 和 val；不用训练集做评估。
- 不在标签未确认前开始长时间训练；先用 20–40 个 session 跑一遍
  `check_dataset.py` 和 1 epoch 烟测。
- 没有 held-out session 的评估结果，不声称检测/关键点精度。

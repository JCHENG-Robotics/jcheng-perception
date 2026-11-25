# 笔记本 GPU 节点的 systemd 管理

> 状态：**2026-11-25 首次安装**。unit 是 **user unit**（`systemctl --user`），
> 跑在操作员笔记本上；机器人侧仍是 5 个 system unit。GPU 节点只消费
> `/perception/uplink/<camera>/image_compressed`（domain 0、CycloneDDS），
> 发布 `/perception/detections/<camera>`，不打开相机设备、不发运动命令。

## 为什么

之前 GPU 节点靠手动 `ros2 launch` 启动；会话结束/终端退出就会被 SIGINT，融合
随即 `gpu_alive=false`、没有检测。现在由用户 systemd 管理，崩溃会自动重启，
重启上限和本地告警与机器人侧一致。

## 安装

```bash
bash deploy/systemd/laptop/install-laptop-gpu.sh
```

安装器写入：

| 路径 | 内容 |
|---|---|
| `~/.config/systemd/user/jcheng-perception-gpu.service` | GPU 节点 unit |
| `~/.config/systemd/user/jcheng-perception-gpu-alert@.service` | 超限告警 |
| `~/.local/lib/jcheng-perception/{gpu-run,gpu-alert}.sh` | 包装/告警脚本 |
| `~/.config/jcheng-perception/gpu.env` | 工作区 / venv / 模型 / DDS 配置 |
| `~/.config/jcheng-perception/gpu_laptop.yaml` | 双相机 YOLO 配置 |
| `~/.config/jcheng-perception/cyclonedds-laptop.xml` | 绑定有线网卡 |
| `~/.local/state/jcheng-perception/` | 告警历史 |

`gpu.env` 默认指向：

- 工作区 `~/jcheng/jcheng-perception/ros2_ws`（Jazzy 构建）
- venv `~/.venvs/jcheng-perception-gpu/bin/python`
- 模型 `/opt/jcheng/models/yolov8s.pt`（不在仓库内）
- `ROS_DOMAIN_ID=0`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

安装前若检测到手动 `laptop_gpu.launch.py` 仍在运行，安装器只 enable 不 start，
避免两个检测节点重复发布。

## 日常操作

```bash
systemctl --user status jcheng-perception-gpu
systemctl --user restart jcheng-perception-gpu
systemctl --user stop jcheng-perception-gpu
journalctl --user -u jcheng-perception-gpu -f
systemctl --user show -p NRestarts --value jcheng-perception-gpu
```

启动后确认：

```bash
ros2 topic echo --once --qos-reliability best_effort /perception/gpu_status
# 或机器人侧 /perception/status: gpu_alive=true
```

## 重启上限 / 告警

与机器人侧相同：`StartLimitBurst=10` / `StartLimitIntervalSec=300`。超限后
unit `failed`，`OnFailure=jcheng-perception-gpu-alert@gpu` 写：

- user journal：`journalctl --user -p err -t jcheng-perception-gpu`
- `~/.local/state/jcheng-perception/last-alert.txt`、`alerts.log`、
  `last-alert-status.txt`

恢复：`systemctl --user reset-failed jcheng-perception-gpu && systemctl --user start jcheng-perception-gpu`。

## 已知边界（重要）

- **只覆盖进程崩溃/退出**。已知的跨机 DDS reader wedge 会让进程活着但
  `frames_received` 冻结；systemd 看不到，需要应用层 watchdog（尚未实现）。
  当前经验规则：edge ingress 重启后，如果 `/perception/status` 的
  `gpu_alive` 仍为 false，`systemctl --user restart jcheng-perception-gpu`。
- 网卡名写死在 `cyclonedds-laptop.xml`（当前 USB 网卡
  `enx00e04ca373c7`）；换网卡后改 XML 并重启 unit。
- 模型权重在 `/opt/jcheng/models/`，不在仓库；换模型改 `gpu_laptop.yaml`
  并重启 unit（`hot_reload` 只处理同路径文件替换）。
- unit 以 user 方式运行，需要 `loginctl enable-linger` 才能在未登录时保持；
  安装器会检查。

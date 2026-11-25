# Split 三进程（2 ingress + fusion）的 systemd 管理

> 状态：**2026-11-25 已在机器人（Pi 5，systemd 252）实机安装并验证**。
> 进程跑在 `ArmPiUltra` 容器里；宿主机 systemd 只管理 `docker exec` 包装脚本，
> 不打开相机设备、不发运动命令。

split 部署原来由 `ros2 launch jcheng_perception_edge split_edge.launch.py`
手动启动，launch 用 `respawn=True` 重启 ingress。现在每个进程一个 unit，
重启策略、重启次数上限和失败告警都由 systemd 负责：

| unit | 进程 | 重启策略 |
|---|---|---|
| `jcheng-perception-ingress@d435i_ext.service` | 外机位 ingress | `Restart=always`, `RestartSec=2` |
| `jcheng-perception-ingress@aurora_wrist.service` | 腕部 ingress | 同上，**每个实例独立计数** |
| `jcheng-perception-fusion.service` | 融合 | 同上 |
| `jcheng-perception-wrist-tf.service` | 腕部动态 TF（总线关节） | 同上 |
| `jcheng-perception-ext-tf.service` | 外机位静态链 `base_link→camera_link` | 同上 |
| `jcheng-perception.target` | 以上五者一起启停 | `WantedBy=multi-user.target` |
| `jcheng-perception-alert@.service` | 失败告警（OnFailure） | oneshot |

两个 TF 发布器不是可选项：没有 `camera_optical_measured` 时腕部观测会被丢弃，
外机位链也需要 `base_link` 才能反投影。原来它们和 launch 一起手动启动，现在
一并纳入 target。`wrist_tf` 只**读**总线舵机（`get_state`），不发运动命令。

ingress 的 4 s `link_stalled` watchdog 逻辑没变：检测到相机 frames/depth
计数停增时，节点主动 `rclpy.shutdown()` 并以 **0** 退出。因此 unit 必须是
`Restart=always`（不是 `on-failure`），重启后是新的 CycloneDDS participant，
与 launch respawn 等价。

## 1. 安装

在机器人宿主机上（文件可先在 `/tmp/jcheng-systemd/`）：

```bash
sudo bash install-split-systemd.sh            # 安装 + 开机自启，不立即启动
sudo bash install-split-systemd.sh --start    # 安装 + 开机自启 + 立即启动
sudo bash install-split-systemd.sh --no-enable
```

安装器写入：

| 路径 | 内容 |
|---|---|
| `/etc/systemd/system/jcheng-perception*.{target,service}` | unit 文件 |
| `/usr/local/lib/jcheng-perception/{ingress-run,fusion-run,wrist-tf-run,ext-tf-run,alert}.sh` | 宿主机包装脚本 |
| `/etc/jcheng-perception/perception.env` | 容器名 / 工作区 / RMW / 可选配置覆盖（首次安装才写入，之后不覆盖） |
| `/var/lib/jcheng-perception/` | 告警历史 |

包装脚本在容器内执行 `ros2 run jcheng_perception_edge perception_edge`
（或 `perception_fusion`），并显式传 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、
`CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml`、`ROS_DOMAIN_ID=0`，
与相机驱动、gateway、GPU 节点保持一致。配置用安装后的
`share/jcheng_perception_edge/config/{cameras,fusion_dual_hardware,frames,view_bias}.yaml`，
可用 `perception.env` 里的 `JCHENG_PERCEPTION_*_CONFIG` 覆盖。

**不要**在这些 unit 运行时再启动 `split_edge.launch.py`：会多出三个同名节点、
重复发布同一个话题。

## 2. 日常操作

```bash
# 整栈
sudo systemctl start jcheng-perception.target
sudo systemctl stop jcheng-perception.target      # PartOf= 会停掉五个进程
sudo systemctl restart jcheng-perception.target

# 单个 ingress（例如只重启外机位，不动腕部和 fusion 状态）
sudo systemctl restart 'jcheng-perception-ingress@d435i_ext.service'

# 状态 / 重启计数 / 日志
systemctl status 'jcheng-perception-*'
systemctl show -p NRestarts,Result --value 'jcheng-perception-ingress@d435i_ext.service'
journalctl -u 'jcheng-perception-*' -f
```

`systemctl is-active` 在 watchdog 退出到重启的 2 s 窗口里会显示
`activating (auto-restart)`，这是正常状态，不是故障。

正常 `systemctl stop` 不会触发告警：`ExecStop` 先写
`/tmp/jcheng-perception-<unit>.stopping` 再杀节点，包装脚本看到标记就把
rclpy teardown 的退出码（1，或 `docker exec` 的 143）翻译成 0，unit 正常
`inactive`。没有标记（崩溃、watchdog 重启）时真实退出码会传给 systemd，
所以崩溃循环仍会撞上 10 次 / 300 s 的上限并触发 `failed` + 告警。

## 3. 重启上限（为什么是 10 次 / 300 秒）

```
StartLimitIntervalSec=300
StartLimitBurst=10
```

- 正常恢复：一次 watchdog 触发 + 2 s 重启，约 6–7 s 一个周期；偶发一次远低于上限。
- 相机彻底失效（USB 掉线、驱动死、`/dev/video*` 被占）：进程会每 ~6 s
  失败一次，10 次约 40–60 s。达到上限后 systemd **停止重启**并把 unit 置为
  `failed`，避免无限循环把 CPU/DDS 占满、掩盖真正的故障。
- 上限用完不会自动恢复；相机修好后必须显式：

```bash
sudo systemctl reset-failed 'jcheng-perception-ingress@d435i_ext.service'
sudo systemctl start  'jcheng-perception-ingress@d435i_ext.service'
```

调参：修改 unit 里的 `StartLimitIntervalSec` / `StartLimitBurst`，
`sudo systemctl daemon-reload` 后生效。监控目的可以把 `StartLimitBurst` 降低
（更早告警），把 `RestartSec` 调大（更少重启）。TF 发布器使用同样的预算：
`wrist_tf` 在舵机服务不可用时不会退出（只重复告警），只有崩溃/配置错误才会
触发上限。

## 4. 告警

超过上限时 `OnFailure=jcheng-perception-alert@ingress-<camera>.service`
（融合是 `@fusion`，TF 是 `@wrist-tf` / `@ext-tf`）触发 `alert.sh`，做三件事：

1. `logger -p user.err`：`journalctl -p err -t jcheng-perception` 可见；
2. 写 `/var/lib/jcheng-perception/last-alert.txt` + `alerts.log`（持久，重启后保留），
   `last-alert-status.txt` 里带该 unit 最后 30 行状态（通常能看到是哪个相机
   topic 停了）；
3. 复制到 `/run/jcheng-perception/`，供后续健康采集器读取。

**这是本地告警**：机器人没有外网，`alert.sh` 不发送远程通知，也不自动重启。
远程感知方式是从笔记本 SSH 读取上述文件，或把 `/run/jcheng-perception/`
接入 `jcheng-ops` 的 `armpi-ops-health` 状态汇总（后续可做）。

## 5. 验证（2026-11-25 实机）

```bash
systemctl list-units --all 'jcheng-perception*'       # target + 5 active
systemctl show -p NRestarts --value 'jcheng-perception-ingress@*'
docker exec ArmPiUltra ps -eo pid,args | grep -E '[p]erception_|[w]rist_tf|[e]xt_tf'
docker exec -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml \
  ArmPiUltra bash -lc 'source /opt/ros/humble/setup.bash; \
    source /home/ubuntu/jcheng-perception/ros2_ws/install/setup.bash; \
    ros2 topic hz /perception/observations/d435i_ext'
sudo systemctl stop jcheng-perception.target
docker exec ArmPiUltra pgrep -af 'lib/jcheng_perception_edge' || echo "clean stop"
```

重启上限 + 告警用一个**无相机**的实例验证，不碰真实相机：

```bash
# 不存在的 camera_filter 会让节点启动即退出 -> 触发 Restart=always 上限
sudo systemctl start jcheng-perception-ingress@selftest.service
# 等 ~40 s，或轮询：
until systemctl is-failed --quiet jcheng-perception-ingress@selftest.service; do sleep 1; done
cat /var/lib/jcheng-perception/last-alert.txt
journalctl -p err -t jcheng-perception -n 20
sudo systemctl reset-failed jcheng-perception-ingress@selftest.service
```

本次实机结果：

- 5 个 unit active；stop 3 s 内 5/5 inactive、`Result=success`、无 marker、
  无告警、无残留容器进程；日志出现
  `stop requested: container exit ... treated as a normal stop`；
- 09:26:42 外机位 ingress watchdog 触发（`frames=75, depth=89`），systemd
  3 s 后重启（restart counter 1）并恢复 `edge ready`；
- `@selftest` 在 10 次重启 / 63 s 后 `failed`，`alert.sh` 写入
  `NRestarts=10`、`label=ingress-selftest`；`reset-failed` 后删除测试实例；
- 验证中约 20 个 `<defunct>` 节点来自早期测试版本；当前 stop/start 不再新增，
  容器重启后清空。
- **安装后重启（2026-11-25）**：target 自动拉起 5/5 unit；fusion/wrist_tf
  0 重启。`ext-tf` 曾在 `start_node` 的 `~/.stop_ros.sh` `kill -9` 窗口内被
  SIGKILL（status 137），unit 已加 `After=/Wants=armpi-vision.service`，本次
  运行已应用但未做第二次重启确认。
- **真实上限事件**：上一 boot 的 10:08:46 外机位 ingress 达到 10 次 / 300 s，
  `Result=start-limit-hit` 并告警；本 boot 25 min 内已 9 次。说明 10/300 会在
  外机位 wedge 高峰期触发——这是上限/告警按设计工作，不是误报；放宽次数属于
  策略选择，当前默认保持 10/300。

完整记录：`evidence/hardware_20261125_systemd_split/`。

## 6. 停止 / 回滚

```bash
sudo systemctl disable --now jcheng-perception.target
sudo bash install-split-systemd.sh --uninstall   # 删 unit 和脚本，保留告警历史
```

回滚后仍可用原来的手动 launch：

```bash
docker exec -u ubuntu ArmPiUltra bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ubuntu/jcheng-perception/ros2_ws/install/setup.bash
  exec ros2 launch jcheng_perception_edge split_edge.launch.py start_tf:=false'
```

## 7. 笔记本电脑 GPU（user unit）

机器人侧栈需要笔记本上的 YOLO 节点配合。`deploy/systemd/laptop/` 把它做成
user unit（`systemctl --user`）：

```bash
bash deploy/systemd/laptop/install-laptop-gpu.sh
systemctl --user status jcheng-perception-gpu
```

- 模型 `/opt/jcheng/models/yolov8s.pt`（不在仓库），配置
  `~/.config/jcheng-perception/gpu_laptop.yaml`；
- `Restart=always`、10 次 / 300 s、`OnFailure` 写
  `~/.local/state/jcheng-perception/last-alert.txt` 等；
- 2026-11-25 验证：`detector loaded`、机器人侧 `gpu_alive=true`、
  stop/start `Result=success`；
- **reader wedge**：`jcheng-perception-gpu-watchdog` 订阅
  `/perception/gpu_status`；单路 `frames_received` 60 s 不增长（另一路仍在
  增长）或状态 30 s 缺失时重启 GPU unit，预算 5 次 / 30 min，超限写
  `last-watchdog-alert.txt`；两路都冻结视为 edge 侧掉线，不重启。实测
  SIGSTOP 后 34 s 检测、重启、4 s 后 `detector loaded`。
- `jcheng-perception-status.timer` 每 60 s 汇总机器人 5 个 unit +
  笔记本 GPU/watchdog + 两侧告警到
  `~/.local/state/jcheng-perception/status.{json,txt}`；`overall` 为
  `ok` / `degraded` / `robot_unreachable`。

完整说明：[`laptop/README.md`](laptop/README.md)。

## 8. 已知边界

- **TF 已纳入 target**：`wrist_tf` 需要厂商工作区里的 `ros_robot_controller_msgs`
  （`/etc/jcheng-perception/perception.env` 的 `JCHENG_PERCEPTION_VENDOR_SETUP`），
  只读总线舵机；`ext_tf` 用 `frames.yaml` 的静态外参。用
  `tf2_echo base_link camera_optical_measured` 验证腕部 TF 在发布。
- 相机驱动仍归厂商 unit（`armpi-rs-depth`、`armpi-vision`）；本 target 只
  `After=`/`Wants=` 它们，不拥有设备。
- 本栈只读：不发布运动命令，不是急停。
- `docker` 容器重启后，包装脚本会等容器最多 60 s；容器不在时 unit 失败并计入
  重启上限。

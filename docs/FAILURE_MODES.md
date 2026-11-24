# 故障模式与降级

## 1. 原则

- **Fail closed**：没有时间匹配的深度/TF 就不产生观测；没有 confirmed 轨迹就发空数组。
- **不阻塞控制链路**：周期状态流 best-effort；编码/网络在工作线程；edge 单线程执行器。
- **可观测**：每次降级都有计数器/状态字段；不要静默。
- **可恢复**：GPU/模型恢复后无需重启 edge；轨迹会重新确认。

## 2. 故障矩阵

| 故障 | 检测信号 | 系统行为 | 恢复 |
|---|---|---|---|
| GPU 节点退出 | `gpu_alive=false`；`detections_received` 停止 | 轨迹 COASTING（0.6 s）→ LOST（1.8 s）；之后发空数组 | 重启 GPU；新检测重新建轨 |
| GPU 过载/队列满 | `GpuStatus.frames_dropped` 增加；延迟上升 | worker 丢最旧帧；不反压 edge | 降分辨率/帧率、换更小模型、检查 GPU |
| 网络丢包 | 上行/检测率下降；`mean_detection_latency` 波动 | 丢帧；时间匹配仍正确（只要帧/深度还在环内） | 检查有线链路/域/RMW |
| 检测延迟 > 缓冲 | `stale_detections`、`frame/depth` 失败计数 | 丢弃；不会拿旧帧冒充 | 增大 `*_buffer_seconds` 或降低延迟 |
| 深度空洞/反光 | `valid_depth_fraction` 低；`occlusion_ratio` 高 | 键点→框中心→网格回退；仍无深度则丢 | 调整光照/相机/量程 |
| 腕部 TF 缺失 | `tf_failures`；wrist 观测消失 | 腕部观测丢弃（无静态回退）；外机位继续 | 恢复 `wrist_tf`（总线服务/厂商消息/`/robot_description`） |
| 腕部 `/joint_states` 与总线不一致 | `tf2_echo` 的相机 z 轴朝上；目标 z 明显高于桌面 | **不要使用**该 TF；`wrist_tf` 只读总线 | 启动 `wrist_tf`；不要用 `camera_optical_calibrated` 的命令缓存父链 |
| 外机位 TF 缺失 | `tf_failures` | 用 `frames.yaml` 静态回退（仅 ext） | 检查驱动/静态 TF |
| 时钟跳变 | 匹配率突变、年龄异常 | 环仍可用但剪枝异常 | chrony 平滑；跳变后重启 edge |
| 模型缺失/损坏 | GPU 每秒错误日志；`frames_failed` | 保留旧模型；无旧模型则无检测 | 放回权重/修路径；热加载 |
| 模型退化（漏检/误检） | 检测率/轨迹数变化；置信度下降 | 系统仍按检测输出，**无法自动识别语义错误** | 影子模型对比、监控、回滚 |
| 同一目标两条轨迹 | `active_tracks` > 目标数 | 合并（6 cm 跨相机 / 2 cm 同相机） | 调 `merge_gate_*`；检查标签 |
| 两个目标过近 | 轨迹数 < 目标数 | 可能错误合并（2 cm 内不可分辨） | 降低合并门限/换更高分辨率 |
| 方向 ±90°/180° 歧义 | yaw 跳变 | 输出一个合法 yaw；规划器需容忍夹爪对称性 | 键点/物体模型消歧 |
| 带宽饱和 | 上行率低于相机率；`frames_rate_limited` | 按 `max_uplink_fps` 限速；队列丢最旧 | 降 JPEG 质量/分辨率/帧率 |
| CPU 饱和 | `timing_log` 回调耗时上升；FPS 下降 | 仍按时间戳丢帧，不阻塞运动层 | 降帧率/分辨率；检查重复节点 |
| 720p edge 在负载高的 Pi 上无界运行 | ping 正常但 SSH banner/网关超时，应用层无响应 | 进程/服务得不到调度 | 限时运行（`timeout -s INT`）、降分辨率/帧率、监控负载 |
| Humble/Jazzy 跨发行版 CycloneDDS | 日志出现 `invalid data size` 序列化告警 | 本次检测/状态流仍正确，但不保证所有类型 | 生产环境优先统一 ROS 发行版 |
| QoS 不匹配 | `ros2 topic echo` 收不到；日志提示 incompatible | 无数据 | 使用 `--qos-reliability best_effort`；不要改回 reliable |
| reliable 回压 | 发布/回调卡顿数秒 | 本版本已改为 best-effort；若自定义 QoS 改回 reliable 会重现 | 恢复 best-effort |
| 相机被其他进程占用 | 驱动无图像/`/dev/video4` busy | edge 无帧；状态 FPS=0 | 单一所有者：网关或 ROS 驱动 |
| 深度/彩色未对齐 | 3D 位置系统性偏移 | 观测有偏；匹配率仍可能正常 | 启用 align_depth_to_color；重新标定 |
| 帧名错误 | TF 查询失败 | 观测丢弃 | 修正 `optical_frame` |
| 两个节点发布同一检测话题 | 框混叠、轨迹抖动 | 无自动检测 | 只允许一个生产检测发布者 |
| 规划器消费过期目标 | 机械臂追旧目标 | 本仓库不校验消费端 | 规划器检查 `header.stamp`；空数组停车 |

## 3. 明确的不可检测项

- 模型把 A 类识别成 B 类（语义错误）且置信度高。
- 相机被遮挡但仍输出“合理”的深度（例如透过玻璃）。
- 标定缓慢漂移（需要定期固定目标复检）。
- 两个物体在 2 cm 内且同标签（低于分辨率）。
- 消费端忽略 `header.stamp` 或空数组语义。

## 4. 恢复操作

```bash
# 清空轨迹（保留环）
ros2 service call /jcheng_perception_edge/reset std_srvs/srv/Trigger

# 强制外机位落点复检
ros2 service call /jcheng_perception_edge/drop_recheck std_srvs/srv/Trigger

# 观察降级
ros2 topic echo --once --qos-reliability best_effort /perception/status
ros2 topic echo --once --qos-reliability best_effort /perception/gpu_status
```

## 5. 安全声明

本仓库是**感知建议源**：不发布运动命令、不做安全校验、没有急停/抱闸。
规划器/安全层必须自行处理空目标、过期目标和异常位姿。软件停止不是安全认证。

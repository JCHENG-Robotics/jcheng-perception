# 验收门禁

> 本文件是 v0.1.0 之前的中文验收门禁记录，保留作参考。
> 当前英文主文档见 [../ACCEPTANCE.md](../ACCEPTANCE.md)。

本仓库不是 M1 骨架，但沿用组织的验收分层。任何“通过”都必须说明是合成、
Mock 还是实机。

## 1. 合并门禁（CI）

| 门禁 | 命令 | 通过条件 |
|---|---|---|
| `m1-gate` | `.github/workflows/ci.yml` | `core-tests`、`simulation`、`ros-build` 全绿 |
| 核心测试 | `python3 -m pytest tests -q` | 0 failed |
| 仿真 | `python3 scripts/simulate_pipeline.py` | 退出码 0；融合 RMSE、遮挡窗口、DROP_RECHECK 检查通过 |
| ROS 构建 | `colcon build --packages-select ...` | 4 包完成 |
| 仓库卫生 | `python3 -m compileall ...`；无模型权重 | 通过 |

## 2. 合成端到端门禁

`ros2 launch jcheng_perception_gpu synthetic_demo.launch.py`：

- 每相机帧率 ≥ 8 Hz（目标 10 Hz）；
- `match_success_rate ≥ 0.95`（排除启动前 2 s）；
- `active_tracks == confirmed_tracks == 1`（单一目标）；
- 融合目标 `source_cameras` 同时包含两路；
- `phase == FINE`，腕权重 > 0.6；
- 无长时间（>1 s）回调停顿。

## 3. 实机门禁（尚未执行）

- Pi 5（ARM64/Humble）构建与运行；
- 相机驱动、深度/彩色对齐、TF 帧名核对；
- 固定目标静止误差、移动目标跟踪误差；
- 外机位遮挡 1 s 的恢复时间；
- GPU 断开 → COASTING → LOST 的时序；
- 30 分钟稳定性、CPU/温度/内存；
- 与 `jcheng_system` 的域隔离、遥测镜像、规划器接口；
- 任何运动集成必须在 `jcheng_safety` 与 `jcheng_motion` 之下，本仓库不直接使能。

## 4. 证据要求

每次实机会话新增 `evidence/<date>_<topic>/`：

- `summary.json`，含 `hardware_claimed: true`；
- 命令、原始日志、状态/目标样本；
- 配置与标定文件的哈希；
- 失败与回滚记录。

合成结果不得标记为硬件验证。

# S1 离线矿石实例分割技术验证报告

> 日期：2026-11-26。范围：S0 设计 + S1 离线技术验证。**未连接机器人、未使用
> 深度、未加入 DDS、未修改冻结基线**。本报告是技术验证，不是真实矿石验收。

## 1. 本轮交付

| 交付 | 位置 | 状态 |
|---|---|---|
| 调研报告 | `docs/ORE_SCENE_RESEARCH.md` | Complete |
| 架构与模块边界 | `docs/ORE_SCENE_UNDERSTANDING.md` | Complete |
| S0–S8 路线图 | `docs/ORE_SCENE_ROADMAP.md` | Complete |
| 场景表达/接口 | `docs/SCENE_REPRESENTATION.md` + `schemas/` | Complete（v1 内部 schema） |
| 实验设计与验收 | `docs/ORE_SCENE_EVALUATION.md` | Complete |
| S1 离线 baseline | `ros2_ws/src/jcheng_perception_scene/` + `scripts/ore_scene.py` | Complete（离线） |
| 合成数据生成器 | `jcheng_perception_scene/synthetic.py` | Complete |
| COCO 适配器 | `jcheng_perception_scene/dataset.py` | Complete |
| 测试 | `tests/test_scene_pipeline.py`（16 个） | 全部通过（总计 93） |
| 证据 | `evidence/ore_scene_s1_20261126/` | 已生成 |

未做：新仓库、`jcheng-interfaces` 修改、运行时节点、深度/3D、真实机器人、
运动、真实矿石精度声明。

## 2. 实现摘要

### 2.1 包与 CLI

新增 ROS-free 的 ament Python 包 `jcheng_perception_scene` v0.1.0：

```text
ros2_ws/src/jcheng_perception_scene/
  jcheng_perception_scene/{types,config,synthetic,dataset,maskops,scene,
                           metrics,pipeline,visualize,cli}.py
  jcheng_perception_scene/backends/{base,classical}.py
  schemas/*.schema.json
```

CLI：`python3 scripts/ore_scene.py <command>`

| 子命令 | 作用 |
|---|---|
| `generate-synthetic` | 生成确定性合成矿石堆 + 逐实例 GT |
| `import-coco` | COCO 多边形 → 内部 manifest + 可见 label map |
| `run` | 数据集 → mask/场景/JSONL/overlay/metrics |
| `evaluate` | 已有 `scenes.jsonl` → metrics |
| `validate` | 校验 scenes.jsonl（schema/禁止字段/关系引用） |
| `run-images` | 无 GT 图像的定性运行（无精度） |
| `selftest` | 空场景/坏图/重叠 mask 三条失败路径 |

退出码：0 成功、2 输入/配置、3 后端/运行、4 输出校验。

### 2.2 后端

默认 `classical_watershed.v1.0.0`：

1. 灰度 Otsu（阈值缩放 0.85，避免丢弃暗色矿石）；
2. 对比度门 + 最小前景比例门（空场景不再产生噪声实例）；
3. 形态学开/闭；
4. 距离变换 + 全局峰值标记（ratio 0.25，min 1.5 px）；
5. 在 5×5 高斯平滑后的 BGR 图上分水岭；
6. 面积过滤（min 120 px，max 20% 图像）；
7. 启发式 score：`0.65·形状项 + 0.35·面积项`，明确标注
   `heuristic_uncalibrated`。

消融 `classical_cc.v1.0.0`：同前景，不做分水岭分裂，用于说明粘连场景的
欠分割。

**没有**引入 torch/ultralytics/SAM 权重；`SegmentationBackend` 协议留给 S1.5。

### 2.3 场景状态

`jcheng.scene_state.v1`：

- 实例：`instance_id`（场景内）、可见 mask/bbox/中心/尺寸、score、
  `confidence_kind`、`field_status`。
- 关系：仅 `adjacent`（2D 接触证据）；遮挡方向 `unknown`。
- 有效性：`valid`、`empty_scene`、`missing_fields`、`failure_reason`、
  `unknowns`（真实质心/完整尺寸/体积/质量/成分/品位/价值）。
- 实例数 >64 时不做邻接，标注 `not_computed_density_limit`（性能边界）。

## 3. 数据

### 3.1 合成测试集

- 生成器 seed **20261127**；36 个非空 + 3 个空场景；640×480。
- 36 个非空场景平衡覆盖 3 光照 × 3 密度 × 4 场景。
- 600 个可见 GT 实例；程序化真值；CC0-1.0。
- 调参使用独立 seed **20261126**（9 场景），测试未参与调参。

### 3.2 公开真实数据

- Mendeley “benchmark for rock blasting” `40_70mm`，CC BY 4.0。
- 49 张 2592×1944，55,370 个 COCO 多边形（`stone`/`Piedras`）。
- 调参：`40_0`、`40_1`；测试：`40_2`–`40_9`（8 张）。
- 标注人工校核比例未核实；可能含 SAM/Mask R-CNN 辅助；重叠多边形被扁平化。
- 因此公开结果只能作为“真实域 baseline”，不是金标准验收。

`transport` 子集已核验：134 张图但只有 COCO 人体关键点模板，**没有矿石
标注**，未用于精度。

## 4. 真实命令

以下命令在本机实际执行（路径以 `<...>` 表示数据集/输出根目录）：

```bash
# 1) 合成测试集
python3 scripts/ore_scene.py generate-synthetic \
  --out <data>/synthetic --scenes 36 --empty-scenes 3 \
  --seed 20261127 --width 640 --height 480

# 2) 合成运行 + 评测
python3 scripts/ore_scene.py run \
  --dataset <data>/synthetic --out <run>/run_synthetic \
  --config config/ore_scene.yaml

# 3) 公开调参/测试导入
python3 scripts/ore_scene.py import-coco \
  --coco <public>/annotations/instances_default.json \
  --images <public>/images/default \
  --out <data>/public_tune --dataset-id mendeley_40_70mm_tune \
  --license CC-BY-4.0 --image-limit 2
python3 scripts/ore_scene.py import-coco \
  --coco <public>/annotations/instances_default.json \
  --images <public>/images/default \
  --out <data>/public_test --dataset-id mendeley_40_70mm_test \
  --license CC-BY-4.0 --image-offset 2 --image-limit 8

# 4) 公开运行
python3 scripts/ore_scene.py run \
  --dataset <data>/public_test --out <run>/run_public_test \
  --config config/ore_scene_public_40_70mm.yaml --no-overlays

# 5) 自检与测试
python3 scripts/ore_scene.py selftest --out <data>/selftest \
  --config config/ore_scene.yaml
python3 -m pytest tests -q
```

## 5. 结果

### 5.1 合成测试集（seed 20261127）

| 指标 | 值 |
|---|---|
| 场景 / GT / 预测 | 39 / 600 / 346 |
| mask AP50 | **0.3682** |
| mask AP50-95 | **0.2114** |
| P@IoU0.5, score0.5 | 0.7513 |
| R@IoU0.5, score0.5 | 0.2467 |
| 计数 MAE / 相对 | 6.51 / 0.4233 |
| 延迟 P50 / P95 / max | 14.4 / 22.3 / 26.1 ms |
| 失败场景 | 0 |
| 空场景 | 3（预测 0） |

分层 AP50：

| 层 | AP50 | 计数 MAE |
|---|---|---|
| sparse | 0.4619 | 1.5 |
| medium | 0.4324 | 5.5 |
| dense | 0.3331 | 14.17 |
| dark | 0.3821 | — |
| normal | 0.4123 | — |
| bright | 0.3265 | — |
| touching | 0.8000（3 场景/15 GT） | — |
| occluded | 0.3570（33 场景/585 GT） | — |

### 5.2 公开真实数据

| 集 | 图 | GT | 预测 | AP50 | AP50-95 | P@0.5 | R@0.5 | 计数 MAE | 延迟 P50/P95 |
|---|---|---|---|---|---|---|---|---|---|
| 调参 | 2 | 2,346 | 890 | 0.0430 | 0.0119 | 0.1339 | 0.0145 | 728.0 | 6.56 / 7.53 s |
| 测试 | 8 | 9,208 | 2,860 | **0.0429** | **0.0122** | 0.2751 | 0.0277 | 793.5 | 4.68 / 8.53 s |

**结论**：经典 baseline 在密集真实碎石上严重欠分割（预测约 360/图，GT 约
1,150/图），AP50 约 0.043，不能作为产品精度。调参与测试几乎一致，说明参数
没有在测试集上调过，但也说明方法上限低。

### 5.3 测试与自检

```text
python3 -m pytest tests -q
93 passed in 2.32s

python3 scripts/ore_scene.py selftest ...
{
  "checks": {
    "empty_scene_ok": true,
    "bad_input_structured": true,
    "overlap_rejected": true
  },
  "passed": true
}
```

新增 16 个测试覆盖：

- 合成确定性/manifest provenance/哈希；
- 空场景 0 实例；
- 端到端 pipeline + metrics smoke；
- 完美预测 AP=1（上界校验）；
- 重叠 mask 拒绝；
- 邻接关系证据；
- 坏图结构化失败；
- 缺失数据集 CLI 退出码 2；
- 配置未知键/不支持后端拒绝；
- CLI validate 拒绝禁止字段；
- CLI selftest；
- 包内无 ROS/设备/网络/模型导入；
- 后端注册表。

## 6. 失败路径与负结果（必须保留）

1. **空场景假目标**：第一版 Otsu 在无目标暗皮带上产生 17–18 个实例；加入
   Otsu 类间对比度门 + 最小前景比例后降为 0。该缺陷已由测试覆盖。
2. **密集欠分割**：合成 dense AP50 0.333、公开测试 AP50 0.043；预测数约为
   GT 的 31%–58%。这是 S1.5 深度后端的直接动机。
3. **Border-color 前景与 local-maxima 种子被否决**：在合成调参集上 AP50
   低于 Otsu-gray + 全局峰值（local maxima 约 0.21–0.26 vs 0.37）。配置中保留
   为可选/消融，不作为默认。
4. **公共 transport 子集无矿石标注**：只有 person_keypoints 模板；未用于精度。
5. **5 MP 场景性能陷阱**：邻接关系 O(N²) 全图计算曾导致单图 >80 s；已改为
   64 实例上限并标注 `not_computed_density_limit`；S3 再做空间索引。
6. **重叠多边形扁平化**：公开 COCO 多边形按标注顺序写入 label map，后写覆盖
   先写；GT 可见面积与 COCO 原义不同，已在文档中声明。

## 7. 当前缺陷与风险

| 缺陷/风险 | 严重度 | 说明 |
|---|---|---|
| 真实矿石精度未验证 | 高 | 无人工金标准；公开标注质量未核实 |
| 密集实例严重欠分割 | 高 | 经典方法上限低；需深度后端 |
| 无深度/3D | 高 | 无真实质心/体积/抓取位姿 |
| score 未校准 | 中 | 不能当概率/操作点保证 |
| 公开数据仅 10 张 | 中 | 统计功效低 |
| 邻接仅 2D、无遮挡方向 | 中 | S3/S4 才补 |
| 5 MP CPU 延迟 4.7–8.5 s | 中 | 不适合在线；离线可用 |
| 权重许可证风险 | 中 | 默认无权重；深度后端需逐项审查 |
| 合成域差距 | 高 | 合成 AP 不可外推真实 |

## 8. 明确未完成

- S1.5：SAM 2/MobileSAM 接入、6 GB 显存实测、同口径对比。
- S2：深度对齐、可见点云/中心/OBB/协方差。
- S3：Scene State v2、3D 关系、空间索引。
- S4：多视角实例关联/融合。
- S5：NBV/主动感知。
- S6：抓取可操作性。
- S7：Applications/Learning/Benchmark 契约与集成。
- S8：真实矿石分拣闭环。
- 真实矿石数据采集/标注、公开数据人工抽检。

## 9. 验收声明

- **S1 离线技术验证：Partially verified**。
  - 已验证：离线图像输入、独立实例 mask、可视化、模型/配置记录、可重复
    CLI、JSONL 导出、空场景/坏输入/重叠/失败路径、合成与公开分源评测。
  - 未验证：真实矿石精度、密集实例召回、深度/3D、GPU 后端、真机。
- 本轮不宣称“矿石识别能力已通过验收”。
- 按授权范围，完成本轮后停止扩展，等待 Board 审查；不自行进入 S2。

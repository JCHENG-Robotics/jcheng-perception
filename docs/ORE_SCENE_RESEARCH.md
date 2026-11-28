# 矿石场景理解调研（S0）

> 核实日期：2026-11-26。本文只记录能追溯到原始论文、官方仓库、官方数据页或原始
> LICENSE 的内容；二手目录站的信息单独标注。未核实项明确写“未核实”，不作为选型
> 依据。本文不声称任何真实矿石精度。
>
> **平台转向（2026-11-28）**：ArmPi Ultra 已 `Frozen`。第 1 节的 ArmPi 调研
> 问题保留为历史记录；后续硬件选型按 PiPER X（6-DoF、CAN、固定单元）复核。
> 文中与 ArmPi 绑定的算力/接口判断不自动适用于新平台。

## 1. 调研范围与方法

调研问题：

1. 矿石/岩石实例分割有哪些公开数据、论文和可复现代码？
2. RGB-D 实例分割、3D 几何估计、多视角关联、场景图、主动感知和抓取可操作性，
   哪些能在 ArmPi Ultra（5-DoF、RTX 3060 6 GB、ROS 2 Humble/Jazzy 混合环境）
   上落地，哪些只能作为设计参考？
3. 许可证是否允许工业产品使用？权重是否可商用？6 GB 显存是否足够？
4. 现有 JCHENG 生态哪些能复用，哪些必须新建？

核实层级：

| 标记 | 含义 |
|---|---|
| **已核实（原始）** | 论文页、官方仓库 LICENSE、官方数据页可直接确认 |
| **已核实（二手）** | 仅目录站/搜索结果确认，未拿到原始文件 |
| **未核实** | 只有论文摘要或转述，代码/数据/许可证未确认 |
| **不可用** | 已确认许可证或访问条件不适合本项目 |

## 2. 结论摘要

1. **S1 不采用 COCO 预训练的 Mask R-CNN/Mask2Former 作为默认基线。**
   Detectron2 代码 Apache-2.0，但其 Model Zoo 权重标注为 **CC BY-SA 3.0**；
   Mask2Former 代码 MIT，但权重标注为 **CC BY-NC 4.0（禁商用）**。两者都不适合
   作为工业交付的默认权重。
2. **YOLO 分割系列（Ultralytics）是 AGPL-3.0。** 仓库现有 `ultralytics` 依赖已
   记录为可选 GPU 后端；若把 YOLO-seg 作为产品默认路径，需要商业授权或接受
   AGPL 义务。S1 不引入新的 YOLO 训练/权重。
3. **SAM 2 / MobileSAM 是当前最干净的“类无关分割”候选**：代码与权重均为
   Apache-2.0；SAM 2.1 Hiera-Tiny 约 39M 参数，官方模型卡标 Apache-2.0，6 GB
   显存有希望但**本轮未实机跑过**。FastSAM 为 AGPL-3.0，EdgeSAM 为非商用
   S-Lab License，均不进入默认路径。
4. **公开矿石/岩石实例数据里，许可证最干净且可直接用于 S1 的是**：
   - ReSyRIS（DLR，CC BY 4.0，3.0 GB，真实+合成岩石实例分割）；
   - Mendeley “benchmark for rock blasting”（CC BY 4.0，7.52 GB，7 个子集）。
   本轮实际下载并核验了其中 `40_70mm`（49 张 2592×1944、55,370 个 COCO 多边形）
   与 `transport`（134 张图，**无矿石标注**）。
5. **S1 采用无权重许可证风险的经典 CV baseline**（Otsu 前景 + 距离变换标记 +
   分水岭），把深度模型作为可插拔后端留给 S1.5/S2。理由不是经典方法精度高，而是
   它能先把数据适配、实例 mask、场景状态、失败路径和评测闭环跑通，且不引入
   AGPL/NC/SA 风险。
6. **场景图、多视角关联、主动感知、抓取可操作性本轮只做设计**。ConceptGraphs
   （MIT）可作设计参考，但其 LLaVA-7B/GPT-4 依赖不适合 6 GB 边缘机；Open3DSG
   为 AGPL-3.0，不进入默认路径。AnyGrasp 闭源且需机器绑定授权；GraspNet
   baseline 仅学术非商用；Contact-GraspNet 推理标称 ≥8 GB 显存。三者都不能直接
   作为 ArmPi 的默认抓取模块。

## 3. 任务定义与难点

矿石场景理解不是“把 YOLO 换成一个矿石类别”这么简单：

| 难点 | 具体表现 | 对设计的影响 |
|---|---|---|
| 实例边界弱 | 碎石颜色/纹理相近，接触边界常被阴影或粉尘覆盖 | 需要实例级 mask 与边界证据，不能只做检测框 |
| 严重遮挡/堆叠 | 可见区域只是碎石的局部，遮挡方向单视角不可判 | 可见中心/可见尺寸不能当真实质心/完整尺寸 |
| 尺度变化大 | 同一画面 40–70 mm 碎石与小块/粉尘共存 | 固定像素阈值脆弱，需要多尺度或学习方法 |
| 域偏移 | 皮带、卡车、堆场、井下光照差异大 | 必须分数据源报告，不能把合成结果当真实精度 |
| 标注昂贵 | 单张图 >1000 个实例，人工逐实例标注成本高 | 需要自动预标注 + 人工校核流程与质量门 |
| 下游安全 | 场景状态可能被 Applications 用于抓取 | 只提供 advisory；运动仍只经 `ExecuteMotion` |

## 4. 数据集调研

| 数据集 | 来源/论文 | 许可证 | 规模/标注 | 可复现性 | 本轮结论 |
|---|---|---|---|---|---|
| **ReSyRIS** | Boerdijk et al., *ReSyRIS – A Real-Synthetic Rock Instance Segmentation Dataset for Training and Benchmarking*, IEEE Aerospace 2023, DOI 10.1109/AERO55745.2023.10115802；Zenodo DOI 10.5281/zenodo.7691201 | **CC BY 4.0**（Zenodo 页面已核实） | 单文件 `resyris.zip` 3.0 GB；36 个 `.obj` 石头模型；真实（Mt. Etna）+ 合成测试集；页面说明支持分割/检测/实例分割 | 高：真实与合成配对，OAISYS 配置随数据发布 | **S1.5 首选外部岩石验证集**；本轮未下载（3 GB），未核验 mask 字段细节 |
| **Mendeley rock blasting** | Ronkin & Reshetnikov, Ural Federal University, Mendeley Data v1, 2026-06-05, DOI 10.17632/v9grvgsrh9.1 | **CC BY 4.0**（页面已核实） | 7 个 zip 共 7.52 GB：open pits、Rocks UAV、bucket excavation、40–70 mm、wagon unloading、transport、asbest；COCO 多边形 | 中高：文件可单独下载，但各子集标注方式不完全统一 | **S1 公开真实数据主来源**；本轮核验 `40_70mm` 与 `transport` |
| Mendeley `40_70mm` 子集 | 同上 | CC BY 4.0 | 49 张 2592×1944；`instances_default.json` 53.8 MB；55,370 个多边形；类别 `stone`/`Piedras`；`iscrowd=0`；`attributes.occluded` 全为 false | 高：多边形可直接栅格化 | 已用于 S1 公开真实数据评测（2 张调参 + 8 张测试） |
| Mendeley `transport` 子集 | 同上 | CC BY 4.0 | 134 张真实皮带图；`annotations/person_keypoints_default.json` 仅 19 KB | 图像可读，但**没有矿石实例标注** | 只能做定性/域检查，不能算精度 |
| OSD / YOLOv8-ORE | Long et al., *YOLOv8-ORE…*, SIViP 19:1280 (2025), DOI 10.1007/s11760-025-04861-7 | **未核实** | 论文称 718 张、11,084 个实例 | **代码/数据未核实**：2026-11-26 访问 `github.com/CQNU-ZhangLab`，仓库列表未显示 YOLOv8-ORE | 不采用，不引用其精度 |
| MineralImage5k | *Computers & Geosciences* 178 (2023) 105414 | 二手目录称 MIT，**原始未核实** | 5,341 张、30,000+ 实例；偏矿物识别/描述，不是矿石堆实例分割 | 中 | 不作为 S1 数据 |
| HF `rock-segmentation-dataset` | HuggingFace `akahukas/…`（2026-06-30 更新） | **CC BY-NC-SA 4.0**（数据集卡已核实） | 142 MB；YOLO 分割标签；挖掘机/捡石场景 | 高 | 非商用/传染性，不进入产品；可做研究对照 |
| FragSAM | Tang et al., *IJMST* 36(6):1167–1188 (2026), DOI 10.1016/j.ijmst.2026.03.017 | 论文开放获取，**代码未找到** | 爆堆/TBM/海滩碎石；mAP50 0.687、~4 FPS（论文口径） | 低：无公开代码 | 仅作方法参考，不引用为实现 |

**数据结论**：

- S1 的“真实矿石”评测必须标注 `source=public`，不能与合成结果合并。
- Mendeley 页面说明部分子集使用 SAM/Mask R-CNN/SAM2.1 辅助分割；`40_70mm`
  的具体人工校核比例未在页面核实。因此本轮只能把它当作“数据集提供的多边形
  参考”，不能称为人工金标准。
- ReSyRIS 的岩石是类地外场景，不等于工业矿石；适合验证“岩石实例分割”泛化，
  不能替代矿石产线数据。

## 5. 方法调研

### 5.1 2D 实例分割

| 方法 | 代表论文/仓库 | 代码许可证 | 权重许可证 | 6 GB 可行性 | 本项目定位 |
|---|---|---|---|---|---|
| Mask R-CNN | He et al., ICCV 2017, arXiv:1703.06870；Detectron2 | **Apache-2.0**（原始 LICENSE 已核实） | **CC BY-SA 3.0**（Model Zoo 页已核实） | 可行 | 研究对照；权重 SA 不适合默认交付 |
| Mask2Former | Cheng et al., CVPR 2022, arXiv:2112.01527 | **MIT**（原始 LICENSE 已核实） | **CC BY-NC 4.0**（MODEL_ZOO 已核实） | 可行 | 研究对照；权重禁商用 |
| YOLOv8/11/26-seg | Ultralytics | **AGPL-3.0**（原始 LICENSE 已核实） | AGPL 条款 | 可行 | 现有 GPU 后端；不新增默认依赖 |
| SAM | Kirillov et al., ICCV 2023, arXiv:2304.02643 | **Apache-2.0**（原始 LICENSE 已核实） | Apache-2.0 | ViT-B 可行 | 类无关 mask 候选；需提示/过滤 |
| SAM 2 / 2.1 | Ravi et al., arXiv:2408.00714 | **Apache-2.0**（原始 LICENSE 已核实） | Apache-2.0（Hiera-Tiny 模型卡 39M 参数） | 有希望，**未实测** | **S1.5 首选深度后端** |
| MobileSAM | Zhang et al., arXiv:2306.14289 | **Apache-2.0**（原始 LICENSE 已核实） | Apache-2.0 | 高 | 轻量备选 |
| FastSAM | Zhao et al., arXiv:2306.12156 | **AGPL-3.0**（原始 LICENSE 已核实） | AGPL | 高 | 不进入默认路径 |
| EdgeSAM | Zhou et al., CVPR 2024, arXiv:2312.06660 | **S-Lab License 1.0，仅非商用**（原始 LICENSE 已核实） | 非商用 | 高 | 不进入产品路径 |

**判读**：许可证是比精度更硬的约束。S1 的默认后端因此选择“无权重”的经典方法；
深度学习后端只通过 `SegmentationBackend` 协议接入，并在依赖审查中逐项记录
代码/权重许可证。SAM 2 是下一步最值得先做 MVE 的候选，但必须实测 6 GB 显存、
自动 mask 生成速度和小目标召回。

### 5.2 3D 几何与点云

| 组件 | 许可证 | 用途 | 结论 |
|---|---|---|---|
| Open3D | **MIT**（原始 LICENSE 已核实） | 点云/网格、OBB、可视化、ICP | S2 可用；注意不要把它带进 Pi 运行时 |
| PCL | **BSD**（原始 LICENSE 已核实） | 点云滤波/分割/配准 | 可选；ROS 侧更重 |
| NumPy 自实现 | BSD-3 | 反投影、PCA、AABB/OBB | S1/S2 优先，减少依赖 |

设计约束：

- 深度反投影必须复用 `jcheng_perception_core.backproject.build_observation` 的
  时间匹配与协方差逻辑，不另写一套“取最近帧”的实现。
- 可见点云中心只能叫 `visible_centroid`；完整质心、体积、质量在 S2 没有多视角/
  先验之前保持 `unknown`。
- 深度有效率、遮挡比例、视差/深度噪声要进入不确定性字段，而不是只给一个分数。

### 5.3 多视角实例关联

| 方法族 | 代表 | 优点 | 风险 |
|---|---|---|---|
| 2D/3D IoU + 匈牙利匹配 | 经典 MOT/多视角融合 | 可解释、无需训练 | 依赖 mask 质量与 3D 定位 |
| 特征嵌入 + 最近邻 | DINO/CLIP 类特征 | 对遮挡更稳 | 权重许可证与算力；本轮未核验具体模型 |
| 视频/可提示分割传播 | SAM 2 | 时序一致性好 | 需要视频流与显存；不是实例身份真值 |
| 场景图/图神经网络 | ConceptGraphs、Open3DSG | 关系推理 | 重依赖；Open3DSG AGPL |

S4 的设计方向：**先用 3D 几何门控 + 匈牙利匹配建立基线**，特征只作为第二证据；
关联结果必须带 `association_confidence` 和 `evidence`，不能把跨视角同 ID 当成
客观真值。

### 5.4 场景图 / Scene State

| 工作 | 许可证 | 可借鉴点 | 限制 |
|---|---|---|---|
| ConceptGraphs (Gu et al., ICRA 2024, arXiv:2309.16650) | **MIT**（仓库页已核实） | 对象节点、开放词表、关系边、RGB-D 增量建图 | 依赖 LLaVA-7B/GPT-4；6 GB 边缘机不现实 |
| Open3DSG (Koch et al., CVPR 2024, arXiv:2402.12259) | **AGPL-3.0**（搜索页已核实） | 开放词表对象/关系 | 许可证不适合默认交付 |
| 3D Scene Graph (Armeni et al., ICCV 2019, arXiv:1910.08811) | 未核验 | 分层场景图概念 | 仅概念参考 |

**决定**：不自研通用 3D 场景图引擎，也不引入上述重依赖。S1/S3 先定义
**轻量、版本化、证据可追溯的 Scene State**（实例 + 2D/3D 证据 + 关系 + 有效性），
S7 再评估是否需要图数据库/查询层。

### 5.5 主动感知 / Next Best View

- 综述：*A structured review and taxonomy of next-best-view strategies for 3D
  reconstruction*（ScienceDirect S2667393225000171，2025；二手检索已核实，
  全文未下载）。
- 可选路线：基于占据/不确定度的信息增益、基于表面覆盖、基于抓取成功率的任务
  驱动 NBV。
- S5 的边界：先在 `jcheng-simulation` 或离线重放中验证；**不直接驱动真机**。
  当前仿真没有深度流和实例真值，S5 前必须先补仿真传感器/真值或使用离线数据。

### 5.6 抓取可操作性

| 方法 | 许可证/访问 | 算力 | 结论 |
|---|---|---|---|
| GraspNet-1Billion baseline | **学术/非营利非商用**（原始 LICENSE 已核实） | 中等 | 不能作为产品默认 |
| AnyGrasp | **闭源二进制 + 机器绑定许可**（官方仓库说明已核实） | 中等 | 许可与部署复杂，暂不采用 |
| Contact-GraspNet | 仓库只有 `License.pdf`，条款未核验；推理标称 **≥8 GB** 显存 | 高 | 6 GB 边缘机不满足 |
| 自研几何抓取评分 | 无第三方权重 | 低 | S6 的可行起点：用可见点云 + 5-DoF 约束做候选评分 |

ArmPi 只有 5-DoF + 夹爪，且无 e-stop、无编码器、无自碰撞模型。S6 的“可操作性”
必须是**建议分数 + 证据 + 未知项**，不能输出“可抓取”布尔真值，更不能直接运动。

## 6. 技术选型决策

| 维度 | 权重 | 经典 CV | YOLO-seg | Mask R-CNN | Mask2Former | SAM 2 |
|---|---|---|---|---|---|---|
| 许可证可交付 | 高 | 无权重风险 | AGPL | 权重 SA | 权重 NC | Apache-2.0 |
| 6 GB 可行 | 高 | 是（CPU） | 是 | 是 | 是 | 待实测 |
| 小目标/密集实例 | 高 | 弱 | 中 | 中 | 中 | 中（需提示/过滤） |
| 矿石域适配 | 高 | 需调参 | 需标注 | 需标注 | 需标注 | 类无关，需下游分类 |
| 可复现/离线 | 高 | 高 | 中（权重） | 中 | 中 | 中（权重） |
| 工程复杂度 | 中 | 低 | 中 | 高 | 高 | 中 |

**S1 决策**：经典 watershed 作为可运行 baseline；`SegmentationBackend` 预留
SAM 2 / MobileSAM / YOLO-seg 适配位。S1.5 的准入条件：许可证审查通过、6 GB
实测通过、在公开/合成集上给出与经典 baseline 的同口径对比。

## 7. 算力与复现评估

| 项 | 本轮实测/核实 |
|---|---|
| 主机 | x86_64，RTX 3060 Laptop 6 GB（S1 未使用 GPU） |
| 640×480 合成图分割 | P50 14.4 ms，P95 22.3 ms（单进程 CPU） |
| 2592×1944 公开图分割 | P50 4.68 s，P95 8.53 s（单进程 CPU） |
| 公开数据规模 | 10 张图、9,208 个可见 GT 实例（测试 8 张） |
| 合成数据规模 | 39 场景、600 个 GT 实例 |
| SAM 2 Tiny | 39M 参数、Apache-2.0；**本轮未下载/未跑**，显存与速度未知 |
| ReSyRIS | 3.0 GB；**本轮未下载** |

## 8. 未核实与风险

1. ReSyRIS 的 mask 字段、类别定义、训练/测试划分未逐项核验。
2. Mendeley `40_70mm` 的标注是否人工校核、SAM 参与比例未核实。
3. OSD/YOLOv8-ORE 的代码与数据在 2026-11-26 未找到公开入口。
4. SAM 2 在 6 GB 上的实际显存、自动 mask 生成参数、小目标召回未实测。
5. 所有“矿石品位/材质/体积/质量”推断均无可靠证据，S1 明确禁止输出。
6. 公开数据的拍摄域（卡车/堆场）与 ArmPi 桌面/皮带场景不同，跨域精度未知。

## 9. 参考来源

- ReSyRIS 论文：https://doi.org/10.1109/AERO55745.2023.10115802
- ReSyRIS 数据：https://doi.org/10.5281/zenodo.7691201
- Mendeley rock blasting：https://doi.org/10.17632/v9grvgsrh9.1
- YOLOv8-ORE：https://doi.org/10.1007/s11760-025-04861-7
- FragSAM：https://doi.org/10.1016/j.ijmst.2026.03.017
- Mask R-CNN：https://arxiv.org/abs/1703.06870
- Mask2Former：https://arxiv.org/abs/2112.01527
- SAM：https://arxiv.org/abs/2304.02643
- SAM 2：https://arxiv.org/abs/2408.00714
- MobileSAM：https://arxiv.org/abs/2306.14289
- FastSAM：https://arxiv.org/abs/2306.12156
- EdgeSAM：https://arxiv.org/abs/2312.06660
- ConceptGraphs：https://arxiv.org/abs/2309.16650
- Open3DSG：https://arxiv.org/abs/2402.12259
- 3D Scene Graph：https://arxiv.org/abs/1910.08811
- GraspNet-1Billion：https://arxiv.org/abs/2011.14206
- AnyGrasp：https://arxiv.org/abs/2212.08333
- Contact-GraspNet：https://arxiv.org/abs/2103.14127
- COCO：https://arxiv.org/abs/1405.0312
- ROS 2 vision_msgs：https://github.com/ros-perception/vision_msgs
- Open3D：https://github.com/isl-org/Open3D
- PCL：https://github.com/PointCloudLibrary/pcl

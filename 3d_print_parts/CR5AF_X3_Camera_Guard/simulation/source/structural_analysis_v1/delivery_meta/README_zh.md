# V2R1 结构仿真工程与复算说明

## 结论和用途

这是冻结 V2R1 原生几何上的实际有限元计算与工程筛查资料。它不是实物试验记录，也不是整套机械臂的认证或设计放行证明。当前版本仍存在快拆自由间隙、相机框较软、保险断裂先后顺序未建立、保险绳没有独立保留相机框等问题。首先阅读随包的中文 PDF 报告及其假设/限制；不要将“完成求解”理解为“结构已合格”。

静力分析覆盖直接底座、D8 卡匣、相机框和定位销；模态及稳态谐响应针对相机框与等效相机质量；底座和卡匣另有理想特征屈曲筛查。机器人末端法兰作为刚性边界。报告没有声称已完成整条机械臂的柔性多体计算、全轨迹/环境碰撞、预紧接触、断裂瞬态、真实绳承接、疲劳、蠕变或热分析。

## 数据如何阅读

- `inputs/`：冻结的 4 个原生 BREP、装配清单、BOM 和源文件 SHA256。原 CAD 没有因本轮分析而改型。
- `fem_core/`：二次四面体线弹性、载荷/力矩分布、场提取及数值验证。
- `mount_fea/`：底座和卡匣三网格单位载荷场；卡匣断后绳耳两网格局部模型。
- `camera_fea/`：相机框三网格静力、已完成 coarse 模态/谐响应、medium 支承敏感性；局部补充算例仅以实际完成文件为准。
- `pin_fea/`：定位销三网格结果；横向受力与轴向倒钩分开建模。
- `service_results/`：由单位场叠加的物理载荷组合及串联子结构位移估计。
- `root_checks/`：梁/模态/屈曲独立验证、组合后处理、图件与报告生成脚本。
- `system_review/`：来源、装配证据审查、螺钉/保险绳计算及接触兼容性节点筛查。
- `final_plot_helpers/outputs/`：实际结果汇总图、CSV 及文件来源哈希。

`results.json` 的 `complete`、`static_results.json`、`modal_results.json` 以及汇总表记录实际完成状态。静态 fine 不表示模态也采用 fine；未收敛的模态不能仅因存在中间文件而作为结果。模态应核查每阶相对残差。资源中止日志保留作诊断，不等于该工况已经完成。

VTU 可用 ParaView 等读取；应力是单元平均值，JSON 原始峰值来自积分点，因此两者最大值不同是预期情况。模态文件按最大节点位移 1 mm 归一化，仅表示振型。NPZ 内包含复算/组合需要的位移、应力张量及自由度映射。请先组合张量/向量，再计算 von Mises 或分位数，不能相加各工况的 von Mises 标量或 p99。

## 已声明的物理假设

统一单位 N、mm、MPa、tonne、s；g=9806.65 mm/s²。材料暂取 E=2000 MPa、ν=0.35、ρ=1.24×10⁻⁹ tonne/mm³，各向同性、全密度、线弹性、小应变。这不是用户材料的实测值；不包含打印层向强度与真实断裂参数。180 g 相机按官方外包络均匀质量近似，其质心 F=(0,112.3050865,+25.3050865) mm，不是测量得到的真实重心。主底座表假设电机/夹爪/工件支路 2 kg、重心 F=(0,0,100) mm，另保存质量敏感性。

1g/3g 指包含重力后的总有效合成加速度，三个方向分别求解，不是三个方向同时各 3g，也不是 3g 后再次叠加 1g。负方向在同一线性边界下由张量反号得到；这不能代替有接触开闭的负方向非线性求解。理想支承、分布载荷映射和远程质量不证明接触压力、滑移或预紧真实。子结构串联位移估计也不是全装联合接触有限元。

屈曲系数是无缺陷线弹性初应力特征值。它不是许用载荷、释放力或安全系数；材料破坏、接触改变和保险颈断裂可能先发生。

## 文件组织与缺少缓存时如何处理

建议将工程与结果分包解压至同一个目标目录，保留各包内的相对目录，结果分包会补全 `structural_analysis_v1/` 内对应目录。不要把多个同名 `results.json` 平铺在一个目录。

全部计算代码、冻结输入及小型结果表在工程包；大网格和场数据可单独分包。完整源装配的 STEP/供应商源模型属于此前 CAD 交付，主文件的 SHA256 见 `inputs/frozen_inputs.json`。全装 95 实例几何审计若要从头重跑，需同时取得此前完整 `recovery_v2` 源工程；仅凭本有限元输入不能重建供应商所有装配来源。现有审计证据及范围已随包保留。

为了缩小下载量，可以省略 `.venv/`、Python 缓存、字体缓存和 `stiffness_K.npz`。K 是可由冻结网格及参数重建的计算缓存。省略 K 时不能直接运行 `resume_checkpoint.py`；使用下面的完整静态运行命令会重新组装并保存 K。请勿把“无 K 缓存”理解为“没有原始位移结果”。

## 环境

本次环境为 Python 3.12，精确直接依赖见 `requirements.txt` 和 `runtime_versions.json`。Linux 环境还需安装 Gmsh/OpenCASCADE 所需图形运行库；无显示器时仍可能需要系统 `libGLU` 库。重新生成中文图和 PDF 需 Noto Sans CJK 字体；现有 PDF 已内嵌字体，阅读无需安装。

将 `requirements.txt` 放在本工程根目录或使用其实际路径：

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -r delivery_meta/requirements.txt
```

以下命令在 `structural_analysis_v1/` 内运行，按顺序执行，不要同时运行多个细网格大矩阵求解。峰值内存依网格、求解器和平台变化；本工程采用分块装配/AMG降低内存。一次运行被系统中止时，应核查结果文件和残差，不能继续把未完成项列为通过。

## 重现数值基准

```bash
.venv/bin/python fem_core/validate_core.py
.venv/bin/python fem_core/validate_chunked_assembly.py
.venv/bin/python root_checks/validate_beam_and_buckling.py
```

基准覆盖拉伸补丁、载荷合力矩、分块装配一致性、梁弯曲、质量/频率单位及几何刚度符号。基准通过证明求解实现的一些基本性质，不证明真实装配边界或材料已被验证。

## 重现底座和卡匣

每行独立执行；`--mesh-only` 可只建网格。源 CAD 哈希不匹配时脚本会停止，不应删除这一检查。

```bash
.venv/bin/python mount_fea/run_mount.py base --level coarse
.venv/bin/python mount_fea/run_mount.py base --level medium
.venv/bin/python mount_fea/run_mount.py base --level fine
.venv/bin/python mount_fea/run_mount.py cartridge --level coarse
.venv/bin/python mount_fea/run_mount.py cartridge --level medium
.venv/bin/python mount_fea/run_mount.py cartridge --level fine
.venv/bin/python mount_fea/run_mount.py cartridge --level coarse --post-break-lug
.venv/bin/python mount_fea/run_mount.py cartridge --level medium --post-break-lug
.venv/bin/python mount_fea/summarize_convergence.py
.venv/bin/python mount_fea/audit_results.py
.venv/bin/python mount_fea/compare_cartridge_regions.py
```

断后绳耳模型使下燕尾自由，固定相机侧载板/柱接口作为等效反力。未实际切除颈，也未模拟断裂或承接时程，其结论仅是这一局部边界下的每 N 弹性敏感性。

## 重现相机模块

相机分析需要冻结框体 BREP、卡匣 BREP、原作者参考 STEP 与 C→F 变换 JSON。若保留历史路径，须同时保留工程包提供的同级 `recovery_v2/camera/` 与 `recovery_v2/mount/` 最小依赖子目录。若交付版已经将这些路径改为 `inputs/`，以 `camera_setup.py` 的实际相对路径为准，文件哈希和几何不变。

```bash
.venv/bin/python camera_fea/camera_setup.py
.venv/bin/python camera_fea/generate_mesh.py coarse
.venv/bin/python camera_fea/generate_mesh.py medium
.venv/bin/python camera_fea/generate_mesh.py fine
.venv/bin/python camera_fea/run_camera_fea.py coarse --export-fields
.venv/bin/python camera_fea/reproduce_coarse_eigsh.py
.venv/bin/python camera_fea/run_camera_fea.py medium --export-fields --support-sensitivity
.venv/bin/python camera_fea/run_camera_fea.py fine --export-fields
.venv/bin/python camera_fea/summarize_camera.py
```

如需追加已经配置的 medium 模态或局部补充算例且 `stiffness_K.npz` 存在，可以使用检查点脚本：

```bash
.venv/bin/python camera_fea/resume_checkpoint.py medium --modes 12
.venv/bin/python camera_fea/resume_checkpoint.py medium --supplemental
```

本交付实际完成的动态结果是 coarse 网格的 12 阶 `eigsh` 直接移位逆模态及对应谐响应，残差检查已通过；没有完成动态网格收敛。`run_camera_fea.py` 后来改成低秩质量算子与 AMG 预条件 LOBPCG 路线，不能将新版路线追记为原 coarse 的执行算法。因此上面先运行 coarse 静态以生成 K，再用 `reproduce_coarse_eigsh.py` 复现原 `eigsh` 方法；新输出写入 `coarse_eigsh_reproduction/`，不会覆盖已交付的 `coarse/` 原结果。该复算入口按已记录的执行实现重建，不冒称原始脚本的逐字节快照；实际算法元数据见 `coarse/EXECUTION_METHOD.json`。

medium 模态曾因资源限制及后续会话中止未完成，以上 medium 命令只是后续复算入口，不表示其结果已获接受。只有实际达到残差要求的模态与相应谐响应可用于结论；迭代次数达到上限并不自动表示已收敛。三个静态网格、medium 支承敏感性和 fine 三轴 1 N 外力已完成；单位夹紧和护柱端力补充工况本轮没有接受结果。其后续脚本即使求解成功，也只给出线弹性灵敏度，不给出拧紧扭矩、真实碰撞位移或断开力。完整范围见 `camera_fea/scope_completion.json`。

## 重现定位销、载荷组合与屈曲筛查

```bash
.venv/bin/python root_checks/run_pin_fea.py --level coarse
.venv/bin/python root_checks/run_pin_fea.py --level medium
.venv/bin/python root_checks/run_pin_fea.py --level fine
.venv/bin/python root_checks/postprocess_service_loads.py --part base --level medium
.venv/bin/python root_checks/postprocess_service_loads.py --part base --level fine
.venv/bin/python root_checks/postprocess_service_loads.py --part cartridge --level medium
.venv/bin/python root_checks/postprocess_service_loads.py --part cartridge --level fine
.venv/bin/python root_checks/run_actual_buckling.py --part base --level coarse
.venv/bin/python root_checks/run_actual_buckling.py --part cartridge --level coarse
.venv/bin/python root_checks/run_actual_buckling.py --part cartridge --level medium
```

如需改变电机支路质量，组合脚本支持 `--motor-mass`，但会覆盖同级输出，应另存完整项目副本后进行参数研究。改变几何、材料本构、载荷面或连接方式需要重新建网格/求解，不能仅更换报告标题。

## 汇总图和报告

```bash
.venv/bin/python final_plot_helpers/plot_final_results.py
.venv/bin/python system_review/audit_fem_methods.py
.venv/bin/python system_review/check_camera_contact_compatibility.py --level medium
```

图表脚本只采纳实际存在且满足所设检查的结果，来源哈希保存在图表 manifest。接触兼容性是相机刚体映射与已解线性框体位移之间的边界节点筛查，不是单边接触有限元，也不是连续表面的无碰撞证明。报告生成还需要所引用的云图、CAD 尺寸图和厂家来源清单；这些图件随工程包提供。只有全部必需的 fine 静态与有效动态结果就绪后才能执行 `root_checks/make_report.py`，脚本不会编造缺失工况。

## 许可和可追溯性

相机参考几何来自 OngXeno 公开模型，而非 Insta360 制造公差图。原作者署名、取得时间和许可记录见随包 `PROVENANCE_AND_LIMITS.md`、`license_response.json` 等。官方资料采用来源 URL/页码/哈希记录；不把厂家试样典型强度当成用户打印件许用值。

请保留原输入、结果元数据与 SHA256；若作任何改型，应另建版本并记录变更。本次报告对应的完整源装配哈希为：

`4ff69b7ea39653468c1949370bb3b173e500666c5146343548b2dc498fc4582f`
此目录只读取已经生成的 FEM 结果绘图，不导入 CAD，不装配矩阵，不调用求解器。

默认执行：
python structural_analysis_v1/final_plot_helpers/plot_final_results.py

指定动态数据为 medium，同时网格对比仅画到 medium：
python structural_analysis_v1/final_plot_helpers/plot_final_results.py --level medium

默认输出到本目录 outputs/，可用 --output-dir 指定另一个目录。

自动选择规则：
1. 动态图/前六模态表选 coarse、medium、fine 中最细且 static、modal、harmonic 三组均完整的同一网格；CSV 必须已存在且时间戳不早于静力检查点。
2. --level 为精确动态级别；若未完成则报错，不默默退回粗网格。
3. 底座/卡匣仅接受 complete=true 且所画三轴算例齐全的 results.json。相机静力只接受三轴 1g、三轴单位力和最差方向均齐全的静力检查点。
4. 网格图不会给未完成级别补点，不会自动宣称已收敛。三部分可以处于不同已完成级别，图角标及 JSON 逐项记录。
5. 图件使用中文 Noto Sans CJK SC，PNG 与 SVG 同时输出。SVG 字形转路径，不依赖接收端字体安装。

图与数据：
01_camera_module_FRF_0to80Hz：Z 输入三阻尼及 5% 阻尼 XYZ 输入；所有实线均为总有效基座加速度幅值 0.1g。
02_response_mesh_sensitivity：底座/卡匣单位远程力广义柔度 2U/P²；相机模块总 1g 质心位移。
03_raw_stress_peak_mesh_sensitivity：同三轴工况的原始积分点峰值，单独显示，不当作已收敛强度。
04_camera_first6_modes_table：前六阶频率和平移有效质量；CSV 含 kg、质量比例、累计比例及残差。
plot_manifest.json：实际处理级别、被排除的未完成数据、所有读取源文件与输出文件哈希。

固定根部、均质材料、刚体相机惯量和假设阻尼的限制在图上保留。相机模块结果不代表整机，也不代表碰撞/断裂/防坠验证。未修改任何原 FEM 报告。
# V2R1 结构仿真与装配校核

**报告标识：SA-V1｜几何版本：V2R1｜分析日期：2026-11-12**

本目录发布项目自有有限元计算源代码、冻结 BREP 输入、关键结果数据和精选分析图。**不上传原始 28 页 PDF、完整报告文本提取或完整内部归档。** 公开摘要见 [DESIGN_VALIDATION_SUMMARY_CN.md](DESIGN_VALIDATION_SUMMARY_CN.md)。

## 目录

- `DESIGN_VALIDATION_SUMMARY_CN.md` / `DESIGN_VALIDATION_SUMMARY_EN.md`：精简技术分析摘要，不是原报告全文。
- `data/`：关键结果、网格等级、模态/谐响应及敏感性 CSV。
- `figures/`：静力云图、模态图、FRF、网格敏感性、螺钉预紧和防坠能量分析。
- `source/structural_analysis_v1/`：有限元源代码、结果 JSON/CSV、冻结自有 BREP、依赖与验证记录。
- `source/recovery_v2/`：复算所需的项目自有 BREP 及 X3 外部参考模型来源说明。
- `reproducibility/README_CN.md`：复算环境、命令和边界条件。

## 关键限制

- 材料参数是均匀各向同性线弹性假设，不是实际打印件实测值。
- 静力模型采用理想化支承；未求解真实预紧、摩擦、单边接触、滑移或断裂。
- 相机模态和谐响应仅有粗网格 12 阶结果，动态网格收敛未完成。
- 屈曲结果是理想线性特征值筛查，不是许用载荷。
- 未证明 D8 保险颈必然先断；当前保险绳不能在所有快拆失效路径下独立保留相机。
- 未进行实物加载、疲劳、释放、跌落承接或碰撞试验。

## 外部模型与许可

相机子模型依赖 OngXeno 发布的 Insta360 X3 社区参考 STEP。该文件未随仓库再分发；请从原作者页面获取并遵守其署名条款，详见 `source/recovery_v2/camera/reference/`。项目自有脚本、文档、数据和 BREP 在仓库根 MIT 许可证允许的权利范围内发布。厂家 CAD 和手册不包含在本目录。

## 复现

先阅读 [复现说明](reproducibility/README_CN.md) 和 [原始工程方法说明](source/structural_analysis_v1/delivery_meta/README_zh.md)。大型稀疏求解按部件顺序串行执行。结果状态以 `camera_fea/scope_completion.json` 和各 `results.json` 为准；命令是复算入口，不代表每个可选工况都已经完成。
# Active Vision Platform（AVP）

本目录包含主动视觉平台的最新 BOM、总装模型、安装说明及配套图片。

## 文件入口

- [完整安装说明](./docs/installation_full.md)：包含机械装配、接线和校准流程。
- [纯机械安装说明](./docs/installation_mechanical.md)：仅包含零部件安装、紧固和机械验收。
- [BOM](./bom/avp_model_BOM.xlsx)：最新版零部件清单。
- [STEP 总装模型](./models/avp_model.stp)：用于三维设计软件中的装配检查和后续修改。
- [STL 总装模型](./models/avp_model.stl)：用于快速预览总装外形。

## 3D 打印零件

- 001 主安装座：[STL](./models/printable_parts/avp_001_main_mount.stl) / [STEP](./models/printable_parts/avp_001_main_mount.stp)
- 002 俯仰支架：[STL](./models/printable_parts/avp_002_tilt_bracket.stl) / [STEP](./models/printable_parts/avp_002_tilt_bracket.stp)
- 004 摄像头支撑件：[STL](./models/printable_parts/avp_004_camera_support.stl) / [STEP](./models/printable_parts/avp_004_camera_support.stp)
- 辅助轴支撑件（零件1）：[STL](./models/printable_parts/avp_aux_axis_support.stl) / [STEP](./models/printable_parts/avp_aux_axis_support.stp)

## 目录结构

```text
avp_model/
├── README.md
├── bom/
│   └── avp_model_BOM.xlsx
├── models/
│   ├── avp_model.stl
│   ├── avp_model.stp
│   └── printable_parts/
│       ├── avp_001_main_mount.stl / .stp
│       ├── avp_002_tilt_bracket.stl / .stp
│       ├── avp_004_camera_support.stl / .stp
│       └── avp_aux_axis_support.stl / .stp
└── docs/
    ├── installation_full.md
    ├── installation_mechanical.md
    └── images/
```

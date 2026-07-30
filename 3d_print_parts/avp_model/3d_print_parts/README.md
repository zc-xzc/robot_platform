# Active Vision Platform（AVP）

本目录包含主动视觉平台的最新 BOM、总装模型、简明机械安装教程及配套图片。

> [!IMPORTANT]
> 本目录中的主动视觉机构是 **2-DOF**：水平旋转（Yaw/Pan）和垂直俯仰（Pitch/Tilt）。BOM 中 `004_3dof_head` 只是历史遗留文件名，不代表第三个运动自由度。

## 文件入口

- [简明机械安装教程](./docs/installation_mechanical.md)：仅包含零部件安装、紧固和机械验收。
- [BOM](./bom/avp_model_BOM.xlsx)：最新版零部件清单。
- [STEP 总装模型](./models/avp_model.stp)：用于三维设计软件中的装配检查和后续修改。
- [STL 总装模型](./models/avp_model.stl)：用于快速预览总装外形。

为避免分发来源授权无法核实的第三方内嵌查看器代码，本目录不提供自包含 HTML 预览文件；请使用 STEP/STP 或 STL 模型。

## 3D 打印零件

- 001 主安装座：[STL](./models/printable_parts/avp_001_main_mount.stl) / [STEP](./models/printable_parts/avp_001_main_mount.stp)
- 002 俯仰支架：[STL](./models/printable_parts/avp_002_tilt_bracket.stl) / [STEP](./models/printable_parts/avp_002_tilt_bracket.stp)
- 004 摄像头支撑件：[STL](./models/printable_parts/avp_004_camera_support.stl) / [STEP](./models/printable_parts/avp_004_camera_support.stp)
- 辅助轴支撑件（零件1）：[STL](./models/printable_parts/avp_aux_axis_support.stl) / [STEP](./models/printable_parts/avp_aux_axis_support.stp)

## 目录结构

```text
3d_print_parts/
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
    ├── installation_mechanical.md
    └── images/
```

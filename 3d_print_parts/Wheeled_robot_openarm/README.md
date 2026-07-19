# Wheeled Robot / OpenArm 3D 打印件

本目录整理自 `Wheeled_robot_openarm.zip` 中的同名 STL 备份，用于保存轮式机器人 OpenArm 相关的主动视觉机构和手部连接件。上传副本保留了原有中文目录、文件名与版本痕迹，未替换或删除仓库中已有的 3D 打印文件。

## 内容概览

| 目录 | 文件数 | 内容说明 |
|---|---:|---|
| `主动视觉/` | 6 | 2-DOF/3-DOF 头部安装件及早期部件 |
| `主动视觉/0714/` | 9 | 0714 版本零件、镜像连接件和装配体导出 |
| `主动视觉/新方案一级旋转费力设计复杂废弃0717/0716STL/` | 5 | 原目录已明确标记为废弃的新方案，仅作历史参考 |
| `主动视觉/旧方案更新/` | 6 | 旧方案更新、旋转支架及不同直径版本 |
| `主动视觉/旧方案更新/0716/` | 2 | 0716 版本 |
| `主动视觉/旧方案更新/0717反向斜撑/` | 4 | 0717 反向斜撑和调距版本 |
| `手部连接件/` | 7 | 宇树 L6 法兰、支撑件和延伸架 |
| **合计** | **39** | 均为 STL 文件 |

## 文件索引

```text
Wheeled_robot_openarm/
├── 主动视觉/
│   ├── 001_2dof_head - G1_head_mount0703.stl
│   ├── 001_2dof_head - G1_head_mount0703test.stl
│   ├── 001_2dof_head - G1_head_mount0706.stl
│   ├── 004_3dof_head - G1_head_mount.stl - 副本.stl
│   ├── 部件1.stl
│   ├── 部件1_1.stl
│   ├── 0714/
│   │   ├── 001_2dof_head - G1_head_mount0703.ipt.stl
│   │   ├── 001_2dof_head - G1_head_mount0703连接件.ipt.stl
│   │   ├── 001_2dof_head - G1_head_mount0703连接件_MIR.ipt.stl
│   │   ├── 011122712.ipt.stl
│   │   ├── 02113122712.ipt.stl
│   │   ├── 031122812.ipt.stl
│   │   ├── 041122713.ipt.stl
│   │   ├── 051142713.ipt.stl
│   │   └── 部件1.iam.stl
│   ├── 新方案一级旋转费力设计复杂废弃0717/
│   │   └── 0716STL/
│   │       ├── 001_2dof_head - G1_head_mount0703.stl
│   │       ├── 001_2dof_head - G1_head_mount0703连接件打印110.stl
│   │       ├── 002.stl
│   │       ├── 1122712_2个.stl
│   │       └── 部件3.stl
│   └── 旧方案更新/
│       ├── 004_3dof_head - G1_head_mount.stl - 副本.stl
│       ├── 004_3dof_head - G1_head_mount.stl - 副本0717.stl
│       ├── 004_3dof_head - G1_head_mount.stl - 旋转支架071701.stl
│       ├── 004_3dof_head - G1_head_mount.stl - 旋转支架071702.stl
│       ├── 004_3dof_head - G1_head_mount.stl - 旋转支架8.95直径.stl
│       ├── 004_3dof_head - G1_head_mount.stl - 旋转支架9.05直径.stl
│       ├── 0716/
│       │   ├── 004_3dof_head - G1_head_mount.stl - 副本.stl
│       │   └── 004_3dof_head - G1_head_mount.stl - 旋转支架.stl
│       └── 0717反向斜撑/
│           ├── 002_2dof_head - G1_head_mount.stl.stl
│           ├── 002_2dof_head - G1_head_mount.stl调距01.stl
│           ├── 002_2dof_head - G1_head_mount.stl调距01缩放孔.stl
│           └── 002_2dof_head - G1_head_mount.stl调距02不缩放小孔.stl
└── 手部连接件/
    ├── 宇树L6法兰1.stl
    ├── 部件1_1.stl
    ├── 部件1_2.stl
    ├── 零件1改支撑25mm.stl
    ├── 零件1改支撑25mm延申架01.stl
    ├── 零件1改支撑25mm延申架02.stl
    └── 零件1改支撑45mm.stl
```

## 去重记录

内容校验发现以下两个文件的 SHA-256 完全相同：

- 保留：`主动视觉/0714/001_2dof_head - G1_head_mount0703连接件.ipt.stl`
- 上传副本中省略：`主动视觉/0714/1122712.ipt.stl`

原始 ZIP 和工作区中的完整解压目录均未修改。

## 校验与使用提示

- 39 个文件均为可解析的二进制 STL，共 48,039 个三角面；文件大小与 STL 中记录的三角面数量一致。
- STL 格式不记录单位。导入切片软件后，应结合设计尺寸确认缩放比例，不能仅凭文件名假定单位。
- `手部连接件/宇树L6法兰1.stl` 检出 86 个零面积三角面；打印前建议使用切片软件或网格修复工具检查并修复。
- `部件1.iam.stl` 来源于装配体导出，切片前请确认是否需要拆分为独立零件。
- 目录名中的日期、`test`、`旧方案更新` 和 `废弃` 均来自原始设计记录。仓库未指定最终量产版本；选择打印件前请先核对实际硬件、孔径和安装方向。
- 为保留设计来源，`.ipt.stl`、`.iam.stl` 和 `.stl.stl` 等原始命名未被改写。

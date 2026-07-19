# Robot Platform

**"Head moves, view moves" — Immersive remote vision for humanoid robots.**

A robotics platform integrating head-tracking active vision, 3D-printable gimbal mounts, and robot simulation models (URDF/SDF). Built around a PICO 4 headset driving a dual-axis STS3032 servo gimbal with an Intel RealSense D415 camera, enabling real-time first-person telepresence.

---

## Overview

| Module | Description |
|--------|-------------|
| `active_vision_dist/` | Active Vision software: head tracking → gimbal control → camera streaming |
| `3d_print_parts/` | 3D-printable mechanical parts for the 2-DOF gimbal assembly |
| `updf_Robotic/` | Robot URDF/SDF models for simulation (MuJoCo, Gazebo, ROS) |

---

## Active Vision System

Real-time immersive remote vision via head-tracking:

```
PICO 4 headset (50 Hz quaternions)
  → WiFi → PC
  → PD controller → STS3032 dual-axis gimbal
  → D415 camera follows
  → Video streamed back to headset
```

### Hardware

| Component | Model | Notes |
|-----------|-------|-------|
| VR Headset | PICO 4 | WiFi link to PC, runs PicoBridge APK |
| Servos (×2) | FeiTech STS3032 12V | ID1=Yaw, ID2=Pitch |
| Serial adapter | URT-1 / URT-2 | USB-to-TTL |
| Camera | Intel RealSense D415 | USB 3.0, gimbal-mounted |
| PC | Windows / Linux | WiFi + USB ports |

### Quick Start

```bash
conda create -n active_vision python=3.10 -y
conda activate active_vision
pip install -r active_vision_dist/requirements.txt

# Windows
cd active_vision_dist/windows
python run.py

# Linux
cd active_vision_dist/linux
python run.py
```

First-time hardware setup requires calibration:
```bash
python run.py --calibrate    # servo limits + center alignment
python run.py --tune-servo   # PID tuning (one-time)
```

For full documentation, see [`active_vision_dist/README.md`](active_vision_dist/README.md) and [`active_vision_dist/README_CN.md`](active_vision_dist/README_CN.md).

---

## 3D Printable Parts

Mechanical assembly for the 2-DOF active vision gimbal, designed to mount on humanoid robots (e.g., Unitree G1).

| Part | Description |
|------|-------------|
| `Active_Vision_platform_001/` | 3-part gimbal assembly (base, connector, camera bracket) with BOM |
| `Wheeled_robot_openarm/` | Wheeled-robot OpenArm STL archive: active-vision mounts and hand/L6 adapters |
| `unitree_l6_joint/` | Unitree L6 joint adapter for robot arm integration |

Assembly instructions and interactive 3D BOM: see [`3d_print_parts/Active_Vision_platform_001/安装说明.md`](3d_print_parts/Active_Vision_platform_001/安装说明.md).

---

## Robot Simulation Models (URDF/SDF)

Pre-built URDF/SDF models for simulation in MuJoCo, Gazebo, and ROS:

| Model | Contents |
|-------|----------|
| `Active_Vision_Platform_001` | 2-DOF gimbal with D415 camera (URDF, SDF, MuJoCo XML) |
| `G1_O6_combined_001` | Unitree G1 humanoid + O6 dual-arm manipulator combined model |
| `unitree_l6_link_001` | Unitree L6 robot arm link model |

Each model includes meshes (`STL`/`OBJ`), launch files, and configuration YAML.

---

## Repository Structure

```
robot_platform/
├── active_vision_dist/          # Active Vision software
│   ├── windows/                 # Windows entry point
│   ├── linux/                   # Linux entry point
│   ├── scservo_sdk/             # FeiTech servo SDK
│   ├── autotune.py              # Auto PID tuning tool
│   ├── requirements.txt
│   ├── README.md / README_CN.md
│   └── TECHNICAL_REFERENCE.md / TECHNICAL_REFERENCE_CN.md
├── 3d_print_parts/              # Gimbal mechanical design
│   ├── Active_Vision_platform_001/
│   ├── Wheeled_robot_openarm/   # OpenArm STL archive and file index
│   └── unitree_l6_joint/
└── updf_Robotic/                # Simulation models
    ├── Active_Vision_Platform_001/
    ├── G1_O6_combined_001/
    └── unitree_l6_link_001/
```

---

## License

This project is for academic research use. See individual subdirectories for details.

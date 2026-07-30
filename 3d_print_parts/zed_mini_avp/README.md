# ZED Mini Active Vision Platform

2-DOF active vision platform for ZED Mini stereo camera, driven by 2x STS3032 serial-bus servos.

ZED Mini 主动视觉云台。适配 ZED Mini 立体相机的二自由度主动视觉云台 (2-DOF yaw/pitch)，使用 2x STS3032 串口总线舵机。

> DOF note: This is a 2-DOF mechanism. Historical filenames containing "3dof" are legacy naming.

## Directory structure

```
zed_mini_avp/
  printable/     -- 3D-printable STL files (14 parts)
  cad/           -- STEP exchange format (9 files)
  src/           -- Inventor CAD source files (.ipt/.iam, 16 files)
  reference/     -- Reference 3D models — camera, servo, URT-2 (10 files)
```

## BOM

| Item | Part | Qty | Type |
|------|------|:---:|------|
| 1 | 001 main mount | 1 | 3D print |
| 2 | 002 tilt bracket | 1 | 3D print |
| 3 | 003 camera plate | 1 | 3D print |
| 4 | ZED camera holder | 1 | 3D print |
| 5 | Camera tab mount | 1 | 3D print |
| 6 | Adapter plate | 1 | 3D print |
| 7 | Part 1 | 1 | 3D print |
| 8 | Feetech STS3032 servo (12V) | 2 | Purchased |
| 9 | ZED Mini stereo camera | 1 | Purchased |
| 10 | URT-2 adapter board | 1 | Purchased |

## Notes

- STL files do not encode physical units. Confirm units/scale in slicer.
- Some parts have multiple versions (v8.9/v9.0/v9.1); pick the matching bore diameter.
- For URDF/simulation models see updf_Robotic/ in the repo root.

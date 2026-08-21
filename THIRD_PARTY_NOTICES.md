# Project Ownership and Third-Party Notices

The repository maintainer identifies the Robot Platform integration, project-specific control code, documentation, CAD/STL/STEP/URDF assets and experimental configuration as project-owned material or material included with authorization. Those portions are offered under the root MIT license to the extent the maintainer holds the applicable rights.

Third-party libraries and external products retain their own licenses and trademarks.

## Included third-party code

| Path | Upstream | License | Notes |
| --- | --- | --- | --- |
| `active_vision_dist/scservo_sdk/` | [FTServo/FTServo_Python](https://gitee.com/ftservo/FTServo_Python) | MIT | FEETECH bus-servo Python SDK. The upstream copyright notice and MIT license are preserved in `active_vision_dist/scservo_sdk/LICENSE`. |

## External dependencies not redistributed here

- [PicoBridge](https://github.com/OpenRobotTech/PicoBridge): obtain its APK/wheel and review its license separately.
- Intel RealSense runtime and `pyrealsense2`: governed by Intel's applicable licenses.
- ROS, Gazebo, MuJoCo and their packages: governed by their respective project licenses.
- PICO, FEETECH, Intel RealSense and Unitree names are used only to describe hardware compatibility. Their trademarks remain with their respective owners.

## Asset scope

If an individual CAD, mesh, URDF, image or document carries a separate notice, that notice takes precedence for that file. The root MIT license grants only rights held by the repository maintainer and does not grant patent, trademark or third-party dataset rights.

## Maintenance rule

Imported components must retain upstream LICENSE/NOTICE files and must be recorded here with source, version, license and modifications.

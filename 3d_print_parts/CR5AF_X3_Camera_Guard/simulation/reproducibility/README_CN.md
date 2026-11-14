# 复现说明

## 环境

Python 3.12；Gmsh 4.15.0；scikit-fem 12.0.1；NumPy 2.3.5；SciPy 1.17.0；meshio 5.3.5；PyAMG 5.3.0。依赖见 `../source/structural_analysis_v1/delivery_meta/requirements.txt`，运行版本见 `runtime_versions.json`。

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -r ../source/structural_analysis_v1/delivery_meta/requirements.txt
```

Linux 还需安装 Gmsh/OpenCASCADE 所需系统图形库（部分无显示环境需要 `libGLU`）。从 `simulation/source/structural_analysis_v1/` 目录运行命令，以保留原始相对路径。大型稀疏求解必须串行执行。

## 数值基准

```bash
python fem_core/validate_core.py
python fem_core/validate_chunked_assembly.py
python root_checks/validate_beam_and_buckling.py
```

## 底座和卡匣

```bash
python mount_fea/run_mount.py base --level coarse
python mount_fea/run_mount.py base --level medium
python mount_fea/run_mount.py base --level fine
python mount_fea/run_mount.py cartridge --level coarse
python mount_fea/run_mount.py cartridge --level medium
python mount_fea/run_mount.py cartridge --level fine
python mount_fea/run_mount.py cartridge --level coarse --post-break-lug
python mount_fea/run_mount.py cartridge --level medium --post-break-lug
python mount_fea/summarize_convergence.py
python mount_fea/audit_results.py
python mount_fea/compare_cartridge_regions.py
```

## 相机模块

相机网格脚本需要项目自有 BREP 和 OngXeno X3 社区参考 STEP。后者未包含在仓库中；按 `source/recovery_v2/camera/reference/PUBLIC_DOWNLOAD_NOTE.md` 从原作者页面获取，并按其署名要求放到指定位置。

```bash
python camera_fea/camera_setup.py
python camera_fea/generate_mesh.py coarse
python camera_fea/generate_mesh.py medium
python camera_fea/generate_mesh.py fine
python camera_fea/run_camera_fea.py coarse --export-fields
python camera_fea/reproduce_coarse_eigsh.py
python camera_fea/run_camera_fea.py medium --export-fields --support-sensitivity
python camera_fea/run_camera_fea.py fine --export-fields
python camera_fea/summarize_camera.py
```

相机动态已接受结果是 coarse 网格 12 阶 eigsh 及对应谐响应；medium 动态结果未完成，不能标为通过。`run_camera_fea.py` 后续的低内存 LOBPCG 路线不是原始 coarse 模态的实际求解算法。

## 定位销、组合载荷与屈曲

```bash
python root_checks/run_pin_fea.py --level coarse
python root_checks/run_pin_fea.py --level medium
python root_checks/run_pin_fea.py --level fine
python root_checks/postprocess_service_loads.py --part base --level medium
python root_checks/postprocess_service_loads.py --part base --level fine
python root_checks/postprocess_service_loads.py --part cartridge --level medium
python root_checks/postprocess_service_loads.py --part cartridge --level fine
python root_checks/run_actual_buckling.py --part base --level coarse
python root_checks/run_actual_buckling.py --part cartridge --level coarse
python root_checks/run_actual_buckling.py --part cartridge --level medium
```

## 边界

这些命令是复算入口，不表示原报告中所有可选工况都已完成。请核对 `camera_fea/scope_completion.json`、`delivery_meta/final_delivery_gate.json` 和各级结果 JSON。当前没有真实材料打印试样、螺钉预紧、非线性接触、疲劳/蠕变、断裂、绳承接或瞬态碰撞试验。
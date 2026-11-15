# Reproduction notes

## Environment

Python 3.12; Gmsh 4.15.0; scikit-fem 12.0.1; NumPy 2.3.5; SciPy 1.17.0; meshio 5.3.5; PyAMG 5.3.0. See `../source/structural_analysis_v1/delivery_meta/requirements.txt` and `runtime_versions.json`.

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -r ../source/structural_analysis_v1/delivery_meta/requirements.txt
```

Linux may also require Gmsh/OpenCASCADE system graphics libraries (including `libGLU` on some headless systems). Run from `simulation/source/structural_analysis_v1/` to preserve the original relative paths. Execute large sparse solves sequentially.

## Numerical benchmarks

```bash
python fem_core/validate_core.py
python fem_core/validate_chunked_assembly.py
python root_checks/validate_beam_and_buckling.py
```

## Base and cartridge

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

## Camera module

The camera meshing scripts require the project-owned BREP and the OngXeno X3 community reference STEP. The latter is not bundled; obtain it from the original author and follow attribution terms. See `source/recovery_v2/camera/reference/PUBLIC_DOWNLOAD_NOTE.md` for the expected path.

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

The accepted camera dynamic result is 12 coarse-mesh eigsh modes and the associated harmonic response. The medium dynamic result was not completed and must not be marked as passed. The later low-memory LOBPCG route in `run_camera_fea.py` is not the algorithm used for the original coarse modal result.

## Pin, service-load combinations and buckling

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

## Scope boundary

These commands are reproduction entry points, not evidence that every optional case in the original report was completed. Check `camera_fea/scope_completion.json`, `delivery_meta/final_delivery_gate.json` and the result JSON files. No physical printed-material coupons, screw preload, nonlinear contact, fatigue/creep, fracture, tether-catch or transient impact tests are available.
# V2R1 structural simulation and assembly review

**Report ID: SA-V1 | Geometry: V2R1 | Analysis date: 2026-11-12**

This directory publishes project-authored FEM source code, frozen project-owned BREP inputs, selected result data and a small set of raster engineering figures. **The original 28-page PDF, full report-text extraction and full internal archive are intentionally not published.** See the concise [design-validation summary](DESIGN_VALIDATION_SUMMARY_EN.md).

## Contents

- `DESIGN_VALIDATION_SUMMARY_*.md`: concise technical summary, not the full original report.
- `data/`: curated key results, mesh levels and camera modal/harmonic CSV tables.
- `figures/fea/`: selected static stress and mode-shape renders; `figures/assembly/` contains the camera screw/recess illustration.
- `source/structural_analysis_v1/`: FEM source modules, selected validation records, frozen project-owned BREP inputs and runtime requirements.
- `source/recovery_v2/`: project-owned BREP dependencies and the external X3 reference-model provenance note.
- `reproducibility/README.md`: environment, commands and modelling boundaries.

Some large raw JSON/CSV fields and vector plots were deliberately omitted from this public release because the text extraction route did not preserve their original bytes reliably. The curated CSV tables and Markdown summary are the public result interface; original raw files remain in the private engineering archive.

## Important limits

- Material properties are isotropic linear-elastic assumptions, not measured printed-part properties.
- Static models use idealized supports; real preload, friction, unilateral contact, slip and fracture are not solved.
- Camera modal/harmonic results use 12 coarse-mesh modes only; dynamic mesh convergence is incomplete.
- Buckling results are ideal linear eigenvalue screens, not allowable loads.
- A D8-first fuse failure sequence is not established; the current tether does not independently retain the camera under every quick-release failure path.
- No physical load, fatigue, release, drop-catch or impact tests have been performed.

## External model and licensing

The camera submodel depends on an X3 community reference STEP by OngXeno. It is not redistributed; obtain it from the original author and follow attribution terms. See `source/recovery_v2/camera/reference/`. Project-authored code, documentation, data and BREP files follow the repository root MIT license only where the maintainer holds the relevant rights. Manufacturer CAD and manuals are not included.

## Reproduction

Read [reproduction notes](reproducibility/README.md) and the [original method notes](source/structural_analysis_v1/delivery_meta/README_zh.md). Run large sparse solves sequentially by component. The accepted status is recorded in `camera_fea/scope_completion.json`; a listed command is a reproduction entry point, not evidence that every optional case was completed.

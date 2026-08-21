# Gripper Forge

A single-file, offline parametric designer for compliant gripper fingers. Open `GripperForge.html` in any browser — no server, no install. Drag the sliders, watch the jaws update live in WebGL, and export binary or ASCII STL ready to print.

## Features

- **Parametric jaws** — 13 parameters: length, tip width, tip lip, wall thickness, ribs, cradle (cup) radius / position / depth, grip bars, fingernail.
- **Identical or mirrored jaws** — design one jaw and print it twice, or tune the two jaws separately.
- **Live 3D preview** — drag to rotate, scroll to zoom, front / side / top orthographic views.
- **In-browser STL export** — binary or ASCII, generated locally (nothing is uploaded).
- **Gripper profiles** — edit the mounting geometry (bolt holes, contact plane, jaw thickness, frame constants) to target any two-jaw gripper; save / load / import / export profiles as JSON.
- **Design persistence** — save / load in the browser, export / import designs as JSON.
- **Material estimate** — opening / reach / material mass with an adjustable density.
- **Quick checks** — in-browser heuristics flag fragile walls, over-deep cradles, thin nails and more.
- **Bilingual** — English / 中文.

## How to print

1. Export the STL(s) and print in TPU 95A. An identical-jaw design is one file printed twice.
2. Print a mount for your gripper in PLA and bolt the fingers to it.

## Adapt to a different gripper

Open the Gripper panel (gear button), edit the mounting fields (bolt hole size and positions, contact face height, jaw thickness, …) and Apply. The full set of frame constants is also editable as JSON for fine control, and profiles can be exported and shared.

## Files

- `GripperForge.html` — the complete tool, self-contained (geometry is computed in the browser; two small open-source geometry libraries are inlined with their license notices).

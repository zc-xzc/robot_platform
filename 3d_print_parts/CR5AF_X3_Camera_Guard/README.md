# CR5AF X3 Camera Guard

A development-stage end-effector camera mounting and protection project for a Dobot CR5AF robot, a reBot B601-DM gripper / DM4310 actuator assembly, and an Insta360 X3 camera.

> **Status: engineering prototype / analysis release. Not safety-certified and not yet validated by physical collision testing.**
>
> This directory is being introduced first as a documentation and provenance landing page. CAD binaries and the full simulation report are not included in this initial PR because the release package contains third-party reference geometry that must not be redistributed without checking its license and provenance.

## Design goals

- Mount an Insta360 X3 camera near the CR5AF end effector at a nominal 45-degree viewing angle.
- Reduce the camera bracket's lever arm compared with the earlier long-bracket design.
- Explore an integrated camera guard and tool-less / quick-release mounting concept.
- Use a replaceable sacrificial interface as a possible collision-management element.
- Provide an independent tether path so a released camera module is not simply dropped.

## Hardware scope

- Robot: Dobot CR5AF (verify exact robot variant and flange revision before fabrication).
- Gripper: reBot B601-DM; the referenced kit uses a DM4310 actuator.
- Camera: Insta360 X3.
- Printed parts: base / direct mount, replaceable fuse cartridge and retaining feature, integrated short 45-degree camera module.

Manufacturer CAD, camera meshes, robot meshes, and other vendor assets are not redistributed here. Obtain those assets from their respective manufacturers and follow their terms.

## Analysis status

A finite-element and structural review was prepared for the V2R1 geometry. The analysis package includes static cases, modal results, harmonic-response data, mesh-sensitivity records, connection/clearance reviews, and a tether-energy sensitivity model.

Important limits:

- Numerical results depend on idealized supports, material assumptions, load directions, and simplified contact/fastener representations.
- The model does **not** establish a guaranteed fuse-first failure sequence.
- The tether must be attached to a load path independent of the quick-release latch; otherwise a latch release can bypass the intended retention function.
- Actual printed-material properties, slicer settings, fastener preload, camera thread engagement, release force, and impact response require physical measurement.
- A linear static or harmonic model is not a substitute for nonlinear transient impact testing.

See [VALIDATION_STATUS.md](VALIDATION_STATUS.md) for the evidence boundary and proposed test gates.

## Planned directory structure

```text
CR5AF_X3_Camera_Guard/
├── README.md
├── README_CN.md
├── VALIDATION_STATUS.md
├── CAD/                 # only project-owned, provenance-cleared CAD exports
├── drawings/            # dimensioned drawings and fit notes
├── BOM/                 # hardware and print-part bill of materials
├── simulation/          # report, scripts, input snapshots, result tables
├── validation/          # mesh, interference, fit and physical-test records
└── images/              # project-owned renders and photographs
```

## Manufacturing notes

Do not print or install a part solely from a rendered assembly image. Confirm units, scale, flange hole pattern, actuator mounting holes, camera screw thread and usable engagement depth, cable access, clearances, print orientation, wall count, infill, and material. Make a low-energy fit-check prototype before operating the robot.

## Licensing and provenance

Project-authored documentation and CAD may be released under the repository's MIT license only where the author holds the necessary rights. Third-party CAD, meshes, trademarks, and manufacturer documents remain subject to their own terms. The presence of a root MIT license does not relicense third-party material.

## Citation

If this project is used in a report or derivative design, cite the repository and identify the exact revision and hardware configuration. Do not describe the current design as collision-safe or production-ready without physical validation.

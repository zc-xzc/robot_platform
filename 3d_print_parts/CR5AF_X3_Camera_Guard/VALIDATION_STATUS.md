# Validation status / 验证状态

## Public analysis package

The V2R1 structural analysis package is published under [simulation/](simulation/):
- [Concise Chinese summary](simulation/DESIGN_VALIDATION_SUMMARY_CN.md) and [English summary](simulation/DESIGN_VALIDATION_SUMMARY_EN.md)
- Selected finite-element, modal, harmonic-response, mesh-sensitivity, tether and fastener figures
- Key CSV/JSON results and FEM source code
- Frozen project-owned BREP inputs and reproduction notes

The original 28-page SA-V1 PDF, full text extraction, internal archive, manufacturer CAD and third-party X3 community STEP are intentionally not redistributed.

## Evidence available

- Linear static FE for the direct base, D8 fuse cartridge, retaining pin and camera module.
- Coarse/medium/fine static mesh records for selected components and convergence data.
- Twelve coarse-mesh camera-module modes and modal-superposition harmonic-response data.
- Ideal linear eigen-buckling screening for the base and cartridge.
- Selected CAD clearance, fastener-stack, tether-energy and post-break lug sensitivity checks.
- Independent beam, patch and chunked-assembly numerical benchmarks.

## What these results do and do not establish

| Topic | Current evidence | Not established |
|---|---|---|
| Static response | Linear static FE under stated idealized boundary conditions | Actual printed-part strength, fatigue or creep life |
| Modal response | 12 camera-module modes on coarse mesh; residual checks recorded | Dynamic mesh convergence or full robot natural frequencies |
| Harmonic response | Modal response for assumed 0.1g sinusoidal input and 2/5/10% damping | Measured robot excitation, random vibration or impact qualification |
| Buckling | Ideal linear eigenvalue screens for base/cartridge | Allowable load, failure load or release threshold |
| Sacrificial fuse | Comparative stress/displacement indicators for selected load directions | Guaranteed D8-first fracture sequence |
| Tether | Geometric and energy sensitivity calculations | Certified tether, lug or anchor strength; validated catch dynamics |
| Assembly | CAD fit/clearance checks for selected geometry and envelopes | All-trajectory collision freedom, physical fit or fastener preload |
| Camera fastener | Nominal 5 mm under-head / 2 mm bearing-layer stack-up | Actual safe X3 thread engagement without measuring socket depth |

## Required physical validation gates

1. **Dimensional inspection:** measure robot flange, actuator mounting pattern, camera base, screw thread, screw under-head length and usable thread depth.
2. **Printed coupon characterization:** record polymer, print orientation, nozzle, wall count, infill, layer height and conditioning; test coupons in relevant directions.
3. **Fastener validation:** verify engagement depth, bottoming clearance, head seating, bearing area and loosening under repeated motion.
4. **Quick-release retention:** measure insertion/removal force and six-axis free play; verify no release under expected service loads.
5. **Fuse calibration:** test repeated sacrificial cartridges in multiple load directions; establish a statistical release-load range relative to service and damage thresholds.
6. **Independent tether test:** attach the tether to the camera-side structure and an independent anchor; test drop energy, stopping distance, lug tear-out and post-release clearance.
7. **Low-energy robot test:** begin with reduced speed, acceleration and workspace; monitor vibration, camera displacement, fastener loosening and cable interference.
8. **Impact test:** only after prior gates pass, use a mass-equivalent dummy and controlled low-energy impacts before installing the camera.

## Release classification

Current classification: **engineering prototype / simulation-supported design review**.

Do not claim: collision-safe, certified, validated for impact, production-ready, or guaranteed sacrificial failure. Update this file only when supporting physical measurements and test records are added.

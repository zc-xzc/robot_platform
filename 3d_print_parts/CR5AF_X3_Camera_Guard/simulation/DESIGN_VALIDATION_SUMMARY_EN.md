# V2R1 structural analysis and design-validation summary

**Report ID:** SA-V1 · **Geometry:** V2R1 · **Original analysis date:** 2026-11-12  
**Purpose:** public engineering summary. This is not the original 28-page PDF and is not a safety certification.

## 1. System and design intent

The design is an end-effector camera mounting/protection concept for a Dobot CR5AF, a B601-DM / DM4310 gripper-actuator branch, and an Insta360 X3. Project-owned parts include a direct base, replaceable D8 fuse cartridge, retaining pin, and shortened integrated 45-degree camera frame. The intent is to explore a replaceable sacrificial interface and an independent retention path that could prevent the camera from falling after a controlled release.

The V2R1 native BREP geometry was frozen for analysis. The assembly audit records 95 named instances and 103 solids, but the finite-element work is **not** a full flexible-body solution of the CR5AF, all robot joints, or the complete gripper. The FE boundary is at the end-effector flange; selected printed parts and a linear serial-substructure estimate were analysed.

## 2. Method, material and load assumptions

- Gmsh / OpenCASCADE curved 10-node tetrahedra; quadratic displacement interpolation in scikit-fem; 3-D small-strain linear elasticity.
- Units: N, mm, MPa, tonne, s; g = 9806.65 mm/s².
- Baseline material assumption: E = 2000 MPa, nu = 0.35, density = 1.24 g/cm³; isotropic, fully dense and linear elastic. These are not measured properties of the actual printed parts.
- Camera: 180 g, nominal envelope 46 × 33.1 × 114 mm; the mass centre is approximated from a uniform envelope, not measured.
- Main actuator/gripper/workpiece branch: assumed 2 kg at z = 100 mm from the flange; additional mass-sensitivity cases were retained.
- 1g / 3g means total effective acceleration including gravity. Axes are solved separately; it is not simultaneous 3-axis 3g and not an extra 1g added to 3g.
- Supports represent idealized engaged interfaces. Real screw preload, frictional slip, unilateral contact opening and fracture are not solved.

## 3. Numerical verification and meshes

Independent benchmarks include a uniaxial patch displacement error of 4.84e-13 mm; a 1 N cantilever displacement of 0.198953 mm versus 0.201620 mm from Timoshenko theory (-1.32%); a beam first mode of 205.272 Hz versus 205.156 Hz from Euler-Bernoulli theory; a linear buckling result of 411.371 N versus 408.512 N from shear-corrected theory; and a relative matrix difference of 9.08e-17 between chunked and full assembly.

| Component | Coarse tetrahedra / DOFs | Medium tetrahedra / DOFs | Fine tetrahedra / DOFs |
|---|---:|---:|---:|
| Direct base | 15,261 / 84,903 | 23,847 / 128,487 | 52,326 / 267,279 |
| D8 cartridge | 11,884 / 64,626 | 26,837 / 137,481 | 68,193 / 329,091 |
| Camera frame | 14,050 / 83,667 | 25,258 / 144,804 | 45,630 / 248,673 |
| Retaining pin | 2,075 / 12,072 | 4,369 / 23,841 | 8,759 / 45,345 |

Displacement/compliance trends support stiffness comparisons; sharp corners, re-entrant shoulders and ideal clamps can still produce mesh-sensitive raw stress peaks. Volume-weighted p95/p99 and raw integration-point peaks must be read separately. A peak stress divided by a vendor filament tensile-strength value is not a valid safety factor for an uncharacterized print.

## 4. Static and substructure results

### Direct base: assumed 2 kg actuator branch plus camera branch

| Total effective acceleration | Max displacement (mm) | Whole-volume p99 (MPa) | Raw peak (MPa) |
|---|---:|---:|---:|
| X 3g | 0.511 | 5.260 | 18.753 |
| Y 3g | 0.511 | 5.263 | 18.812 |
| Z 3g | 0.656 | 2.936 | 13.302 |

### D8 cartridge: camera branch

| Total effective acceleration | Max displacement (mm) | Whole-volume p99 (MPa) | D8-region p99 (MPa) |
|---|---:|---:|---:|
| X 3g | 0.198 | 5.325 | 9.345 |
| Y 3g | 0.038 | 0.828 | 2.006 |
| Z 3g | 0.232 | 4.373 | 9.792 |

The D8 minimum neck diameter is 8 mm and its nominal bending section modulus is 50.265 mm³. The nominal section formula gives 19.894 MPa/(N·m), but does not include print-layer fracture, transition stress concentration or loading-rate effects; it cannot predict a guaranteed breakaway force.

### Camera frame

| Total effective acceleration | Camera CoG displacement (mm) | Frame max nodal displacement (mm) | Whole-volume p99 (MPa) |
|---|---:|---:|---:|
| X 3g | 0.862 | 1.287 | 2.016 |
| Y 3g | 0.827 | 1.198 | 1.489 |
| Z 3g | 3.251 | 6.062 | 5.489 |

For +Z total 1g on the fine static mesh, camera CoG displacement is 1.084 mm and the frame maximum nodal displacement is 2.021 mm. These are different measurement points. The +Z direction is the main camera-frame stiffness concern.

### Serial substructure estimate

The base, cartridge and camera-frame 6-DOF interface responses were mapped to the same assumed camera CoG. The +Z total-1g serial estimate is 1.908 mm; the worst-direction total-1g estimate is 1.966 mm. The corresponding 3g value of 5.899 mm is a small-motion linear extrapolation, not a full assembly contact FE solve including preload, slip or robot-arm flexibility.

## 5. Quick-release, contact and pin checks

- The Ø3.6 mm post in a 4.2 mm slot gives 0.6 mm nominal lateral diametral clearance.
- The nominal lift clearance between the stud-head underside and camera-frame foot is 0.3 mm.
- Changing the idealized camera-frame support from a broad foot patch to two stud-projection regions increased the same 1g CoG displacement by about 2.9% / 8.0% / 17.1% in X/Y/Z. This shows sensitivity to the interface state; neither idealization is a strict bound on real contact response.
- A camera-contact compatibility screen checked 28,494 boundary nodes over 12 independent ±X/±Y/±Z, 1g/3g cases. No new internal nodes were found at sampled guard-post/lens-height locations; the conservative lower-bound clearance was about 0.280 mm. This is not a continuous-surface, all-trajectory or full-robot collision proof.
- Pin transverse and barb-axial cases were modelled separately. Insertion, friction, snap-back and pull-out were not simulated, so no retention-force rating is established.

## 6. Modal and steady-state harmonic response

The camera frame plus 180 g equivalent camera mass has 12 accepted modes on the **coarse** mesh only. The first six frequencies are 12.673, 25.721, 61.041, 61.860, 69.135 and 117.369 Hz; the maximum relative eigen residual is about 2.49e-9. Retained effective-mass ratios in X/Y/Z are 90.3% / 92.9% / 87.6%. Dynamic mesh convergence was not completed; 12.673 Hz is not the first mode of the complete robot or full end-effector assembly.

The harmonic response uses an assumed 0.1g sinusoidal input, modal superposition, and assumed damping ratios of 2% / 5% / 10%. For Z input, the camera CoG peak displacements near the main resonance are about 2.621 / 1.048 / 0.526 mm. These are conditional sensitivity results, not a measured robot vibration spectrum or impact response. “Steady state” here means periodic mechanical excitation; thermal steady-state and long-term creep were not analysed.

## 7. Buckling, fasteners and tether path

Ideal linear eigen-buckling screening gives a first positive factor of 69.681 for the coarse base under -Z total 3g; the cartridge factors are 75.189 coarse and 67.407 medium. The cartridge coarse-to-medium difference is about 10.35%, so it is not adequately mesh-converged. These factors are not allowable loads, safety factors or release factors.

Nominal fastener stack-up: CR5AF M6×16 projection about 6.9 mm; actuator M3×10 projection about 2.6 mm, including the 2.4 mm OEM rear cover; X3 camera screw assumes 5 mm under-head length and a 2 mm bearing layer, giving 3 mm nominal projection. Actual usable X3 thread depth and the user's screw-head geometry were not measured. Rear recess: 44×32 mm, R4, 6 mm deep, with a 2 mm remaining bearing layer.

Preload sensitivity uses T≈KFd with K=0.15–0.25 as an assumption, not an installation torque recommendation. Plastic bearing, washer bending and creep were not solved with real preloaded contact.

The moving tether lug has a Ø4.2 mm hole, 2.5 mm thickness and only 0.9 mm minimum net edge; the fixed lug is 7.5 mm thick with 1.9 mm net edge. The current tether connects the fixed base to the moving cartridge: it can retain the camera only if D8 breaks first and the quick-release remains engaged. If the mushroom stud, key slot or short hook fails first, the tether does not independently constrain the released camera frame. The tether-energy example uses an assumed 0.25 kg mass, 20 mm slack fall and 0.5 m/s initial velocity; tension varies strongly with stop distance and is not a real impact upper bound or tether rating.

## 8. Engineering conclusion and next steps

**Current status: V2R1 is suitable as an engineering screening and redesign baseline, but it has not passed collision-protection or whole-system safety validation.** The critical issue is not whether one isolated component has a low stress result; it is whether the release sequence and camera-retention path behave as intended.

Recommended order:
1. Attach the tether to a retention path independent of the quick-release latch and able to constrain the camera frame; avoid a rigid bypass across the D8 fuse.
2. Add repeatable preload/wedge/anti-play features to the quick-release and measure six-axis free play and retention after repeated cycles.
3. Increase stiffness at the short 45-degree arm, camera foot and base receiver while preserving tool access.
4. Calibrate the replaceable fuse using repeated specimens printed with the same material, orientation and process. Demonstrate normal loads below the minimum release threshold and the maximum release threshold below damage thresholds of the camera and non-fuse parts.
5. Measure screw under-head length, usable thread length, camera socket depth, preload window, plastic crushing/relaxation and creep.
6. Use a mass-equivalent dummy for six-axis static loading, modal/sweep testing, release and tether-catch tests before controlled low-energy impact trials.

## 9. Reproduction and data

This repository preserves project-owned BREP inputs, FEM scripts, result JSON/CSV, selected SVG/PNG figures and runtime requirements. The camera submodel depends on an X3 community reference STEP by OngXeno; it is not redistributed here. Obtain it from the original author and follow the attribution terms. The original 28-page PDF, full text extraction and internal archive are intentionally not published. See the reproduction notes and camera scope-completion manifest for accepted and incomplete cases.
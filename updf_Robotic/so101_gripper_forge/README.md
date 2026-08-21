# SO-101 × Gripper Forge — compliant fingers in a full arm URDF

This folder integrates fingers designed in [Gripper Forge](../../3d_print_parts/gripper_forge/GripperForge.html)
into a complete SO-101 arm model, so you can simulate and inspect your custom
compliant fingers on the arm in RViz, Isaac Sim, MoveIt, or any URDF loader.

The SO-101 gripper is a two-jaw parallel gripper driven by a single motor
(`gripper`). The standard jaws can be replaced by compliant TPU fingers — the
flexible, ribbed "soft-fin" design. **That compliant finger is exactly what
Gripper Forge generates**, and Gripper Forge exports it in the SO-101 mount
frame (same bolt pattern, same contact plane), so the STL drops straight into
the URDF's finger links.

## Files

| File | Purpose |
| --- | --- |
| `so101_gripper_forge.urdf` | Complete arm + gripper model. Fingers are real Gripper Forge geometry. |
| `meshes/so101_finger_universal.stl` | Example left jaw, exported from the Gripper Forge *Universal* preset. |
| `meshes/so101_finger_universal_right.stl` | Mirrored copy used by the right jaw. |
| `inject_gripper_forge.py` | Re-point any arm URDF at your own Gripper Forge fingers. |

## Quick start

Open [Gripper Forge](../../3d_print_parts/gripper_forge/GripperForge.html),
design a finger, export the STL, then inject it:

```bash
python inject_gripper_forge.py \
    --urdf path/to/your/so101.urdf \
    --left-finger my_left.stl \
    --out my_so101_fingers.urdf
```

The right jaw is derived by mirroring the left STL (one file, printed twice).
Use `--right-finger` to supply a different jaw, `--left-link` / `--right-link`
to name the finger links explicitly, and `--mesh-xyz` / `--left-rpy` /
`--right-rpy` to force a mesh origin if your base URDF places the fingers
elsewhere.

To (re)generate the bundled model from scratch:

```bash
python inject_gripper_forge.py --new --out so101_gripper_forge.urdf
```

## The URDF model

- **Joints** are named to match the SO-101 control convention:
  `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`,
  `gripper` (left jaw) plus a `gripper_right` joint that `mimic`s the gripper
  joint in reverse, so one value opens and closes both jaws symmetrically.
- **Fingers**: the left link turns the Gripper Forge export 180°, the right
  link uses a mirrored mesh, so the two contact faces point at each other
  across the grasp axis. The mount face sits at the gripper mount plane.
- **Arm link dimensions are approximate** (simple box/cylinder visuals). The
  official SO-101 URDF lives in a gated repository; running the script on the
  official file keeps its exact link origins and only swaps the finger meshes.

## Adapting to other arms

The same recipe works for any two-jaw gripper with replaceable fingers:

1. **Identify the two finger links** and their joint in the target arm's URDF.
2. **Measure the finger mount** on the physical jaw: bolt-hole diameter and
   Y-positions, bolt inset, base length, and the height of the contact face
   above the mount plane.
3. Enter those numbers into Gripper Forge's **Gripper panel** (gear button) to
   build a gripper profile, then design and export the fingers.
4. Run `inject_gripper_forge.py` against the target URDF with the exported STL,
   adjusting `--mesh-xyz`/`--left-rpy`/`--right-rpy` so the mesh origin matches
   the target finger link origin.

The workflow suits any arm whose gripper exposes a flat, replaceable finger
mount: the SO-101 / SO-ARM100 (this folder), hobby parallel-jaw grippers, and
research arms with printable fingertip blanks. For grippers without a
replaceable jaw face, print a small adapter plate that bolts to the jaw and
carries the Gripper Forge finger.

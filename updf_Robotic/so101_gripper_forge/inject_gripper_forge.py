#!/usr/bin/env python3
"""
inject_gripper_forge.py — put Gripper Forge fingers into a robot-arm URDF.

Two ways to use it:

  1. Re-point an existing arm URDF (e.g. the official SO-101 URDF) at your own
     Gripper Forge fingers:

       python inject_gripper_forge.py \
           --urdf so101.urdf \
           --left-finger  my_left.stl \
           --right-finger my_right.stl \
           --out so101_gripper_forge.urdf

     The script finds the gripper finger links, copies the STLs into ./meshes,
     and rewrites their <mesh> filename + scale. It does NOT touch the link
     origins, so the official model keeps its exact finger placement. Use
     --left-link/--right-link to name the links explicitly, and
     --mesh-xyz/--left-rpy/--right-rpy to force an origin if needed.

  2. Start from the reconstructed model shipped next to this script
     (the arm uses approximate link dimensions, but the finger geometry is
     exact Gripper Forge output):

       python inject_gripper_forge.py --new --out so101_gripper_forge.urdf

     This writes a copy of so101_gripper_forge.urdf from this folder, and
     re-points the finger meshes if --left-finger/--right-finger are given.

Finger meshes from Gripper Forge are exported in the mount frame (base at +x,
tip at -x, contact face at +y, in mm). When a base URDF already places the
fingers correctly, only the filename and a mm->m scale are needed.
"""

import argparse
import os
import shutil
import struct
import sys
import xml.etree.ElementTree as ET

MM_TO_M = "0.001 0.001 0.001"

# Names tried, in order, when --left-link/--right-link are not given.
LEFT_PATTERNS = ["gripper_left_finger", "left_finger", "finger_left", "gripper_left"]
RIGHT_PATTERNS = ["gripper_right_finger", "right_finger", "finger_right", "gripper_right"]

# Finger links in the reconstructed model (so --new can re-point them).
DEFAULT_LEFT_LINK = "gripper_left_finger"
DEFAULT_RIGHT_LINK = "gripper_right_finger"


def find_link(root, name):
    for link in root.findall("link"):
        if link.get("name") == name:
            return link
    return None


def autodetect_link(root, patterns):
    names = {l.get("name") for l in root.findall("link")}
    for p in patterns:
        for n in names:
            if n == p or n.endswith("/" + p):
                return n
    return None


def replace_meshes(link, mesh_path, scale, xyz, rpy):
    """Point every <mesh> in the link's visuals/collisions at mesh_path."""
    replaced = 0
    for mesh in link.iter("mesh"):
        mesh.set("filename", mesh_path)
        if scale:
            mesh.set("scale", scale)
        replaced += 1
    if (xyz or rpy) and replaced:
        for geom_host in ("visual", "collision"):
            el = link.find(geom_host)
            if el is None:
                continue
            origin = el.find("origin")
            if origin is None:
                origin = ET.SubElement(el, "origin")
            if xyz:
                origin.set("xyz", xyz)
            if rpy:
                origin.set("rpy", rpy)
    return replaced


def mirror_stl(src, dst):
    """Write a copy of src mirrored about the X axis (for the opposite jaw)."""
    with open(src, "rb") as f:
        data = f.read()
    if len(data) < 84:
        raise ValueError(f"not a binary STL: {src}")
    n = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + n * 50:
        raise ValueError(f"expected binary STL (found {n} tris, {len(data)} bytes)")
    out = bytearray(data)
    off = 84
    for _ in range(n):
        for _ in range(4):  # normal + 3 vertices
            x = struct.unpack_from("<f", out, off)[0]
            struct.pack_into("<f", out, off, -x)
            off += 12
        off += 2
    with open(dst, "wb") as f:
        f.write(bytes(out))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--urdf", help="base arm URDF to modify")
    src.add_argument("--new", action="store_true",
                     help="start from the reconstructed model shipped with this script")
    ap.add_argument("--left-finger", help="Gripper Forge finger STL for the left jaw")
    ap.add_argument("--right-finger", help="STL for the right jaw (default: mirror of left)")
    ap.add_argument("--mirror-right", action="store_true", default=True,
                    help="derive the right jaw by mirroring the left STL")
    ap.add_argument("--no-mirror-right", action="store_false", dest="mirror_right",
                    help="do not derive the right jaw from the left")
    ap.add_argument("--left-link", help="link name of the left finger")
    ap.add_argument("--right-link", help="link name of the right finger")
    ap.add_argument("--mesh-dir", default="meshes",
                    help="subfolder (relative to the output URDF) for copied STLs")
    ap.add_argument("--scale", default=MM_TO_M, help="mesh scale (default mm->m)")
    ap.add_argument("--mesh-xyz", help="force mesh <origin> xyz (e.g. '0.0095 0 0')")
    ap.add_argument("--left-rpy", help="force left mesh <origin> rpy")
    ap.add_argument("--right-rpy", help="force right mesh <origin> rpy")
    ap.add_argument("--out", default="so101_gripper_forge.urdf")
    args = ap.parse_args()

    if args.new:
        here = os.path.dirname(os.path.abspath(__file__))
        base = os.path.join(here, "so101_gripper_forge.urdf")
        if not os.path.exists(base):
            sys.exit(f"--new requires so101_gripper_forge.urdf next to this script (not found: {base})")
        default_left, default_right = DEFAULT_LEFT_LINK, DEFAULT_RIGHT_LINK
        # The reconstructed model already positions fingers; only re-point them.
        args.mesh_xyz = args.mesh_xyz or "0.0095 0 0"
        args.left_rpy = args.left_rpy or "0 0 3.141592653589793"
        args.right_rpy = args.right_rpy or "0 0 0"
    else:
        base = args.urdf
        default_left = default_right = None
        if not os.path.exists(base):
            sys.exit(f"base URDF not found: {base}")

    tree = ET.parse(base)
    root = tree.getroot()

    left_link = args.left_link or autodetect_link(root, LEFT_PATTERNS) or default_left
    right_link = args.right_link or autodetect_link(root, RIGHT_PATTERNS) or default_right
    if not left_link and not right_link:
        sys.exit("no finger links found; pass --left-link/--right-link explicitly")

    out_dir = os.path.dirname(os.path.abspath(args.out)) or "."
    mesh_dir = os.path.join(out_dir, args.mesh_dir)
    os.makedirs(mesh_dir, exist_ok=True)

    # ---- stage the STL files and remember their URDF-relative references ----
    refs = {}
    if left_link and args.left_finger:
        name = os.path.basename(args.left_finger)
        shutil.copyfile(args.left_finger, os.path.join(mesh_dir, name))
        refs["left"] = f"{args.mesh_dir}/{name}"
    if right_link:
        if args.right_finger:
            name = os.path.basename(args.right_finger)
            shutil.copyfile(args.right_finger, os.path.join(mesh_dir, name))
        elif args.left_finger and args.mirror_right:
            name = "finger_right.stl"
            mirror_stl(args.left_finger, os.path.join(mesh_dir, name))
        else:
            name = None
        if name:
            refs["right"] = f"{args.mesh_dir}/{name}"

    # ---- rewrite the meshes ----
    if left_link and "left" in refs:
        link = find_link(root, left_link)
        if link is None:
            sys.exit(f"left link not found: {left_link}")
        n = replace_meshes(link, refs["left"], args.scale, args.mesh_xyz, args.left_rpy)
        print(f"left  finger: {left_link} -> {refs['left']} ({n} mesh(es))")
    if right_link and "right" in refs:
        link = find_link(root, right_link)
        if link is None:
            sys.exit(f"right link not found: {right_link}")
        n = replace_meshes(link, refs["right"], args.scale, args.mesh_xyz, args.right_rpy)
        print(f"right finger: {right_link} -> {refs['right']} ({n} mesh(es))")

    tree.write(args.out, encoding="utf-8", xml_declaration=True)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Interactively inspect the converted AVP model in MuJoCo."""

import argparse
import math
from pathlib import Path
import platform
import sys
import time

import mujoco
import mujoco.viewer


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--sweep",
        action="store_true",
        help="Continuously sweep the second joint between -90 and +90 degrees.",
    )
    args = parser.parse_args()

    model_path = Path(__file__).resolve().with_name("avp_model.xml")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    if platform.system() == "Darwin" and Path(sys.executable).name != "mjpython":
        print(
            "MuJoCo viewer on macOS must be started with mjpython.\n"
            f"Run:\n  mjpython {Path(__file__).resolve()} --sweep"
        )
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.time()
        while viewer.is_running():
            if args.sweep:
                data.qpos[1] = math.radians(90.0) * math.sin(
                    (time.time() - start) * 0.7
                )
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1.0 / 60.0)


if __name__ == "__main__":
    main()

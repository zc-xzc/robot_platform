"""MuJoCo viewer for avp_model."""
import mujoco
import mujoco.viewer
import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))

model = mujoco.MjModel.from_xml_path("avp_model.xml")
data = mujoco.MjData(model)

print(f"Model: {model.nbody} bodies, {model.ngeom} geoms, {model.njnt} joints")
print(f"Gravity: {model.opt.gravity}")
print("Launching viewer... (close window to exit)")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

"""Regenerate the native MuJoCo reference for the browser free-cube release test."""
import json
from pathlib import Path

import mujoco
import numpy as np

web = Path(__file__).resolve().parents[1]
rows = []
for scene in ("vector-free", "vector-contact42-free"):
    model = mujoco.MjModel.from_xml_path(str(web / "public/robot" / (scene + ".xml")))
    for axis in (0, 1):
        for sign in (-1, 1):
            data = mujoco.MjData(model)
            joint = data.joint("cube_root")
            angle = np.deg2rad(15.76)
            joint.qpos[3:] = [np.cos(angle/2), *(sign*np.sin(angle/2)*np.eye(3)[axis])]
            mujoco.mj_forward(model, data)
            cube = model.body("cube").id
            geoms = np.flatnonzero(model.geom_bodyid == cube)
            zmin = min(data.geom_xpos[g, 2] - abs(data.geom_xmat[g, 6:9]) @ model.geom_size[g]
                       for g in geoms)
            joint.qpos[2] += 1e-9 - zmin  # Avoid an ambiguous touching contact at exactly zero.
            pose = joint.qpos.tolist()
            trace = []
            while data.time < .15:
                mujoco.mj_forward(model, data)
                trace.append([data.time, data.xmat[cube, 8], *data.sensor("cube_accel").data])
                mujoco.mj_step(model, data)
            rows.append(dict(scene=scene, axis=axis, sign=sign, pose=pose, trace=trace))

path = web / "test/fixtures/cube-landing.json"
path.write_text(json.dumps(rows, separators=(",", ":")) + "\n")

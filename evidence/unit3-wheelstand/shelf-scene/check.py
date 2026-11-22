"""Compile the exported contact scene and place the bodies at the W13 sighting."""
import csv
import math
from pathlib import Path

import mujoco as mj
import numpy as np

here = Path(__file__).resolve().parent
m = mj.MjModel.from_xml_path(str(here.parents[2]/"web/public/robot/vector2/vector-contact42-free-shelf.xml"))
d = mj.MjData(m)
with (here/"W13-registered.csv").open() as f:
    for row in csv.DictReader(f):
        joint = "root" if row["object"] == "chassis" else "cube_root"
        q = m.joint(joint).qposadr[0]
        yaw = float(row["yaw_rad"]) / 2
        d.qpos[q:q+2] = [float(row[k])*.001 for k in ("x_mm", "y_mm")]
        d.qpos[q+3:q+7] = [math.cos(yaw),0,0,math.sin(yaw)]
mj.mj_forward(m,d)
for x,y,expected in [(45,120,"shelf_"),(150,170,"floor"),(-90,0,"dock_apron"),(-134,0,"dock_bed")]:
    geom = np.zeros(1,np.int32)
    distance = mj.mj_ray(m,d,np.array([x,y,250.])*.001,np.array([0.,0.,-1.]),
                         np.array([1,0,0,0,0,0],np.uint8),1,-1,geom)
    name = m.geom(geom[0]).name
    assert name.startswith(expected), (name,expected)
    print(f"{name} surface height {250-1000*distance:.2f} mm")
for _ in range(1500):
    mj.mj_step(m,d)
ranges = [1000*d.sensor("cliff_"+end+"_"+side).data[0]
          for end in ("front","back") for side in ("left","right")]
assert all(0 < value < 20 for value in ranges)
assert 20 < d.body("cube").xpos[2]*1000 < 24
print("W13 at rest, cliff beam lengths (mm)", np.round(ranges,2))

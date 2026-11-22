"""Release the free cube at R5's settled tilt on the stationary printed fork."""
import csv
import itertools
import sys
from pathlib import Path
from xml.etree import ElementTree

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

here = Path(__file__).resolve().parent
repo = here.parents[2]
sys.path[:0] = [str(repo), str(repo/"evidence/unit3-printed-fork-contact")]
from replay_links import add_links, rail_paths, require_printed_fork
from simulacrum import params

tree = ElementTree.parse(sys.argv[1])
require_printed_fork(tree)
add_links(tree, repo)
model = mj.MjModel.from_xml_string(ElementTree.tostring(tree.getroot(), encoding="unicode"))
v = params.values(params.load(model="vector_2"))
robot = np.genfromtxt(here/"state.csv.gz", delimiter=",", names=True)
hold = (robot["sec"] >= 29) & (robot["sec"] < 31.5)
height = np.mean(robot["lift"][hold])
base_x, _, base_z = v["lift_base_position_mm"]
reach = base_x + np.sqrt(v["lift_arm_length_mm"]**2 - (height-base_z)**2)
wrist = np.array([reach, 0, height]) * .001
rails, rq = rail_paths(repo, *np.zeros((3, 1)), np.array([height]))
row = next(csv.DictReader((here/"carry-attitude.csv").open()))
up = np.array([float(row[f"mean_{axis}_g"]) for axis in "xyz"])
up /= np.linalg.norm(up)
z = np.array([0., 0., 1.])
attitude = Rotation.from_quat(np.r_[np.cross(up, z), 1 + up@z])
floor = model.geom("table").id
with (here/"static-support.csv").open("w") as f:
    out = csv.writer(f, lineterminator="\n")
    out.writerow(["yaw_deg","forward_mm","left_mm","clearance_raise_mm","first_floor_s","final_tilt_deg","final_direction_error_deg"])
    for yaw, dx, dy in itertools.product(range(0,360,45), [-3,0,3], [-3,0,3]):
        d = mj.MjData(model)
        d.mocap_pos[model.body("fork").mocapid[0]] = wrist
        for i in range(2):
            idx = model.body(f"rail{i}").mocapid[0]
            d.mocap_pos[idx], d.mocap_quat[idx] = rails[0,i], rq[0]
        q = d.joint("cube_root").qpos
        q[:3] = wrist + .001*(np.array(v["cube_carry_offset_mm"]) + [dx,dy,0])
        q[3:] = (Rotation.from_euler("z",yaw,degrees=True)*attitude).as_quat(scalar_first=True)
        initial_z = q[2]
        for tick in range(401):
            q[2] = initial_z + tick*.0002
            mj.mj_forward(model,d)
            if all(c.dist >= 0 for c in d.contact):
                break
        else:
            raise RuntimeError("No clear release height")
        first = None
        for _ in range(round(2.5/model.opt.timestep)):
            mj.mj_step(model,d)
            if first is None and any(floor in c.geom for c in d.contact):
                first = round(d.time,4)
        final = Rotation.from_quat(q[3:],scalar_first=True).inv().apply(z)
        out.writerow([yaw,dx,dy,round(.2*tick,1),first,np.degrees(np.arccos(np.clip(final[2],-1,1))),np.degrees(np.arccos(np.clip(up@final,-1,1)))])

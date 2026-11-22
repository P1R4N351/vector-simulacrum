"""Replay the physical R5 approach starting with the cube on the shelf."""
import csv
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

here = Path(__file__).resolve().parent
repo = here.parents[2]
sys.path[:0] = [str(repo), str(repo/"evidence/unit3-printed-fork-contact")]
from replay_links import add_links, rail_paths, require_printed_fork
from simulacrum import params

def read(name):
    return np.genfromtxt(here/name, delimiter=",", names=True)

tree = ET.parse(sys.argv[1])
require_printed_fork(tree)
add_links(tree, repo)
m = mj.MjModel.from_xml_string(ET.tostring(tree.getroot(), encoding="unicode"))
v = params.values(params.load(model="vector_2"))
p = read("approach.csv")
s = read("sightings.csv")
s = s[s["sec"] < 0][-1]
assert np.all(p["origin"] == s["origin"])
t = np.arange(-.5, 8.3, m.opt.timestep)
x, y, pitch, height = [np.interp(t, p["sec"], p[k]) for k in ("x","y","pitch","lift")]
yaw = np.interp(t, p["sec"], np.unwrap(p["yaw"]))
bx, _, bz = v["lift_base_position_mm"]
reach = bx + np.sqrt(v["lift_arm_length_mm"]**2 - (height-bz)**2)
wrist = .001*np.column_stack([x+reach*np.cos(yaw), y+reach*np.sin(yaw), height])
rails, link_q = rail_paths(repo, x, y, yaw, height)
axis = .001*np.column_stack([x, y, np.full_like(x,v["wheel_front_radius_mm"])])
tilt = Rotation.from_rotvec(np.column_stack([pitch*np.sin(yaw), -pitch*np.cos(yaw), 0*t]))
wrist = axis + tilt.apply(wrist-axis)
rails = np.stack([axis+tilt.apply(rails[:,j]-axis) for j in range(2)],axis=1)
fork_q = (tilt*Rotation.from_euler("z",yaw[:,None])).as_quat(scalar_first=True)
link_q = (tilt*Rotation.from_quat(link_q,scalar_first=True)).as_quat(scalar_first=True)
d = mj.MjData(m)
cube = d.joint("cube_root").qpos
h = Rotation.from_quat([s[k] for k in ("qx","qy","qz","qw")]).as_euler("xyz")[2]
cube[:] = [s["x"]*.001,s["y"]*.001,.0222,np.cos(h/2),0,0,np.sin(h/2)]
fork = m.body("fork").mocapid[0]
links = [m.body(f"rail{i}").mocapid[0] for i in range(2)]
floor = m.geom("table").id
traces = []
for i, tick in enumerate(t):
    d.mocap_pos[fork], d.mocap_quat[fork] = wrist[i], fork_q[i]
    d.mocap_pos[links], d.mocap_quat[links] = rails[i], link_q[i]
    if i == 0:
        mj.mj_forward(m,d)
        assert all(c.dist >= 0 for c in d.contact)
    mj.mj_step(m,d)
    if i%10 == 0:
        lean = np.degrees(np.arccos(np.clip(1-2*np.sum(cube[4:6]**2),-1,1)))
        traces.append([tick,cube[2]*1000,lean,any(floor in c.geom for c in d.contact),
                       any(floor not in c.geom for c in d.contact),
                       cube[0]*1000-s["x"], cube[1]*1000-s["y"]])
a = np.array(traces)
np.savetxt(here/"pickup-replay.csv",a,fmt="%.5f",delimiter=",",comments="",
           header="sec,center_height_mm,tilt_deg,floor_contact,robot_contact,dx_mm,dy_mm")
c = read("cube.csv.gz")
with (here/"actions.csv").open() as f:
    start = float(next(r["sec"] for r in csv.DictReader(f) if r["action"].startswith("pickup") and r["kind"]=="begin"))
clock = c["sec"]-start
sel = (clock >= t[0]) & (clock <= t[-1])
fig, ax = plt.subplots(3,1,sharex=True,figsize=(8,7),layout="constrained")
ax[0].scatter(clock[sel],np.degrees(np.arctan2(np.hypot(c["x"][sel],c["y"][sel]),c["z"][sel])),s=3,label="Physical apparent tilt")
ax[0].plot(a[:,0],a[:,2],label="Simulated geometric tilt")
ax[1].plot(t,height,label="Recorded wrist height")
ax[1].plot(a[:,0],a[:,1],label="Simulated cube center")
observed = read("sightings.csv")
ax[2].scatter(observed["sec"],np.hypot(observed["x"]-s["x"],observed["y"]-s["y"]),
              label="Camera pose estimate")
ax[2].plot(a[:,0],np.hypot(a[:,5],a[:,6]),label="Simulated center")
for panel in ax:
    panel.axvspan(29-start,31.5-start,color=".7",alpha=.25)
    panel.legend()
ax[0].set_ylabel("Degrees")
ax[1].set_ylabel("Height (mm)")
ax[2].set(xlabel="Seconds from pickup command",ylabel="Planar displacement (mm)")
fig.savefig(here/"pickup-replay.png",dpi=150)

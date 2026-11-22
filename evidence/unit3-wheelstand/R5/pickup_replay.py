"""Replay the physical R5 approach starting with the cube on the shelf."""
import argparse
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
sys.path.insert(0, str(repo/"evidence/unit3-cube-accelerometer"))
from pickup_support import contact_load
from pickup_patch import collect

def read(name):
    return np.genfromtxt(here/name, delimiter=",", names=True)

parser = argparse.ArgumentParser()
parser.add_argument("fixture")
parser.add_argument("--output", type=Path, help="Directory for this replay’s generated evidence")
parser.add_argument("--substeps", type=int, default=1, choices=(1,2,4))
parser.add_argument("--freeze-at-raise", choices=("planar", "pitch", "both"),
                    help="Hold selected chassis motion at its value when the wrist first rises 1 mm")
args = parser.parse_args()
if args.freeze_at_raise and not args.output:
    parser.error("Use --output for counterfactual runs to preserve the recorded baseline")
tree = ET.parse(args.fixture)
require_printed_fork(tree)
add_links(tree, repo)
m = mj.MjModel.from_xml_string(ET.tostring(tree.getroot(), encoding="unicode"))
m.opt.timestep /= args.substeps
v = params.values(params.load(model="vector_2"))
p = read("approach.csv")
s = read("sightings.csv")
s = s[s["sec"] < 0][-1]
assert np.all(p["origin"] == s["origin"])
t = np.arange(-.5, 8.3, m.opt.timestep)
x, y, pitch, height = [np.interp(t, p["sec"], p[k]) for k in ("x","y","pitch","lift")]
yaw = np.interp(t, p["sec"], np.unwrap(p["yaw"]))
if args.freeze_at_raise:
    onset = np.flatnonzero(height > height[0] + 1)[0]
    channels = {"planar": (x, y, yaw), "pitch": (pitch,), "both": (x, y, yaw, pitch)}
    for channel in channels[args.freeze_at_raise]:
        channel[onset:] = channel[onset]
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
loads = []
patch = []
cube_id, fork_id = m.body("cube").id, m.body("fork").id
weight = m.body_mass[cube_id] * np.linalg.norm(m.opt.gravity)
for i, tick in enumerate(t):
    d.mocap_pos[fork], d.mocap_quat[fork] = wrist[i], fork_q[i]
    d.mocap_pos[links], d.mocap_quat[links] = rails[i], link_q[i]
    if i == 0:
        mj.mj_forward(m,d)
        assert all(c.dist >= 0 for c in d.contact)
    mj.mj_step(m,d)
    if 1.5 <= tick < 3.8:
        overlap_mm = max((max(0., -1000*c.dist) for c in d.contact
                          if fork_id in m.geom_bodyid[c.geom] and cube_id in m.geom_bodyid[c.geom]), default=0.)
        loads.append([tick, height[i], *contact_load(m,d,cube_id,floor,fork_id)/weight, overlap_mm])
        collect(m,d,cube_id,fork_id,tick,m.opt.timestep,patch)
    if i % (10*args.substeps) == 0:
        lean = np.degrees(np.arccos(np.clip(1-2*np.sum(cube[4:6]**2),-1,1)))
        traces.append([tick,cube[2]*1000,lean,any(floor in c.geom for c in d.contact),
                       any(floor not in c.geom for c in d.contact),
                       cube[0]*1000-s["x"], cube[1]*1000-s["y"]])
# Finer solver steps are a sensitivity check on the same measured pose trajectory.
a = np.array(traces)
output = args.output or (here if args.substeps == 1 else here/f"substeps-{args.substeps}")
output.mkdir(exist_ok=True)
np.savetxt(output/"pickup-replay.csv",a,fmt="%.5f",delimiter=",",comments="",
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
fig.savefig(output/"pickup-replay.png",dpi=150)

loads = np.array(loads)
np.savetxt(output/"contact-load.csv",loads,delimiter=",",fmt="%.6f",comments="",
           header="sec,wrist_mm,shelf_z_weight,fork_shell_z_weight,fork_rubber_z_weight,link_z_weight,lift_horizontal_weight,fork_overlap_mm")
# Integrate signed impulses by wrist-motion phase; contact normals point into the cube.
moving = np.flatnonzero(height > height[0]+1)[0]
raised = np.flatnonzero(height > height.max()-1)[0]
fig, axes = plt.subplots(2,1,sharex=True,figsize=(8,5.5),layout="constrained")
window = 10*args.substeps
averaged = loads[:len(loads)//window*window].reshape(-1,window,loads.shape[1]).mean(axis=1)
axes[0].plot(averaged[:,0],averaged[:,2],label="Shelf")
axes[0].plot(averaged[:,0],averaged[:,3:6].sum(axis=1),label="Fork and links")
axes[0].axhline(1,color=".5",ls=":")
axes[0].set_ylabel("Vertical force / cube weight")
axes[0].legend()
points = np.array([r[2:] for r in patch])
ticks = np.array([r[0] for r in patch])
dots = axes[1].scatter(ticks,points[:,2],s=np.clip(points[:,4]*4e4,1,80),
                       c=points[:,3],cmap="coolwarm",vmin=-1,vmax=1)
fig.colorbar(dots,ax=axes[1],label="Upward normal component")
axes[1].set(ylabel="Height on cube (mm from center)",xlabel="Seconds from pickup command")
for panel in axes:
    panel.axvspan(t[moving],t[raised],color=".6",alpha=.15)
fig.savefig(output/"contact-load.png",dpi=150)
with (output/"contact-impulse.csv").open("w") as stream:
    writer = csv.writer(stream,lineterminator="\n")
    writer.writerow(["phase","start_s","end_s","normal_Ns","normal_up_Ns","friction_up_Ns",
                     "normal_fraction_on_rubber","normal_fraction_upward_over_half","normal_fraction_below_center"])
    for name, lo, hi in [("approach",1.5,t[moving]),("raise",t[moving],t[raised]),("raised",t[raised],3.8)]:
        selected = [r for r in patch if lo <= r[0] < hi]
        total = sum(r[6] for r in selected)
        fractions = [sum(r[6] for r in selected if predicate(r))/total if total else ""
                     for predicate in (lambda r: "rubber" in r[1],lambda r: r[5]>.5,lambda r: r[4]<0)]
        writer.writerow([name,round(lo,4),round(hi,4),total,sum(r[7] for r in selected),
                         sum(r[8] for r in selected),*fractions])

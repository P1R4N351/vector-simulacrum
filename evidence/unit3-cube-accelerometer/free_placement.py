"""Test free cube support during the physical placement's measured wrist trajectory."""
import csv
import itertools
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

here=Path(__file__).resolve().parent
repo=here.parents[1]
sys.path[:0]=[str(repo),str(here.parent/"unit3-printed-fork-contact")]
from replay_links import add_links, rail_paths
from simulacrum import params
from decode import decode

def main():
    tree=ET.parse(sys.argv[1])
    fork=tree.find(".//body[@name='fork']")
    assert len(fork.findall("geom"))==58
    add_links(tree,repo)
    model=mj.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding="unicode"))
    v=params.values(params.load(model="vector_2"))
    actions=list(csv.DictReader((here/"action-times.csv").open()))
    zero=float(next(a["pod_unix_s"] for a in actions if a["action"]=="place"))
    robot=np.genfromtxt(here/"robot.csv",names=True,delimiter=",")
    dt=model.opt.timestep
    times=np.arange(-.4,2.85,dt)
    # Stand still for the support check, then drive the fork from recorded lift height.
    heights=np.interp(np.maximum(times,0),robot["pod_unix_s"]-zero,robot["lift_mm"])
    base_x,_,base_z=v["lift_base_position_mm"]
    reach=base_x+np.sqrt(v["lift_arm_length_mm"]**2-(heights-base_z)**2)
    wrist=np.column_stack([reach,np.zeros_like(times),heights])*.001
    static=np.zeros_like(times)
    rails,rq=rail_paths(repo,static,static,static,heights)
    ids=[model.body(f"rail{i}").mocapid[0] for i in range(2)]
    forkid=model.body("fork").mocapid[0]
    pkt,_=decode(here/"pickup.tsv")
    stamp=np.array([p[0]*1e-9-zero for p in pkt])
    g=np.array([np.frombuffer(p[3],"<i2").reshape(3,3).mean(0)/8192 for p in pkt])
    # Settled pre-turn force defines cube tilt. Its compass heading is unobserved.
    vertical=np.median(g[(stamp>-3.6)&(stamp<-2)],axis=0)
    vertical/=np.linalg.norm(vertical)
    z=np.array([0.,0.,1.])
    q=np.r_[np.cross(vertical,z),1+vertical@z]
    attitude=Rotation.from_quat(q)
    scores=[]
    seating=[]
    fig,ax=plt.subplots(figsize=(8,4))
    sel=(stamp>=-.4)&(stamp<2.85)
    ax.plot(stamp[sel],np.degrees(np.arctan2(np.linalg.norm(g[sel,:2],axis=1),g[sel,2])),"k.",label="Cube apparent tilt")
    for yaw,dx,dy in itertools.product(range(0,360,90),(-3,0,3),(-3,0,3)):
        d=mj.MjData(model)
        pos=d.joint("cube_root").qpos
        pos[:3]=wrist[0]+.001*(np.array(v["cube_carry_offset_mm"])+[dx,dy,0])
        pos[3:]=(Rotation.from_euler("z",yaw,degrees=True)*attitude).as_quat(scalar_first=True)
        # Second initialization: lower the cube under gravity from the first
        # non-overlapping height above the same XY. Keep the wrist stationary.
        probe=mj.MjData(model)
        probe.qpos[:]=d.qpos
        probe.mocap_pos[forkid]=wrist[0]
        probe.mocap_pos[ids]=rails[0]
        probe.mocap_quat[ids]=rq[0]
        origin=pos[2]
        for lift in range(81):
            probe.joint("cube_root").qpos[2]=origin+.001*lift
            mj.mj_forward(model,probe)
            if not any(c.dist < 0 for c in probe.contact):
                break
        else:
            raise ValueError("Could not initialize a clear cube")
        first_floor=None
        for j in range(round(.4/dt)):
            mj.mj_step(model,probe)
            if first_floor is None and any(model.geom("table").id in c.geom for c in probe.contact):
                first_floor=j*dt
        final=probe.joint("cube_root").qpos
        lean=np.degrees(np.arccos(np.clip(1-2*np.sum(final[4:6]**2),-1,1)))
        seating.append([yaw,dx,dy,lift,1000*(origin-final[2]),lean,first_floor])
        floor_t=None
        tilt=[]
        for i,t in enumerate(times):
            d.mocap_pos[forkid]=wrist[i]
            d.mocap_pos[ids]=rails[i]
            d.mocap_quat[ids]=rq[i]
            if i==0:
                mj.mj_forward(model,d)
                overlap=max([0.]+[-1000*c.dist for c in d.contact])
            mj.mj_step(model,d)
            if i==round(.4/dt)-1:
                drop=1000*(wrist[0,2]+.001*v["cube_carry_offset_mm"][2]-pos[2])
            if floor_t is None and any(model.geom("table").id in c.geom for c in d.contact):
                floor_t=t
            if i%10==0:
                tilt.append(np.degrees(np.arccos(np.clip(1-2*np.sum(pos[4:6]**2),-1,1))))
        scores.append([yaw,dx,dy,overlap,drop,floor_t])
        if dx==dy==0:
            ax.plot(times[::10],tilt,label=f"Free, yaw {yaw}°",lw=1)
    ax.axvline(0,color=".5",ls=":")
    ax.set(xlabel="Seconds from place command",ylabel="Tilt (°)",ylim=(-1,100))
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(here/"free-placement.png",dpi=150)
    with (here/"free-placement.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["yaw_deg","forward_offset_mm","left_offset_mm","initial_overlap_mm","center_drop_at_command_mm","first_floor_s"])
        w.writerows(scores)

    with (here/"free-seating.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["yaw_deg","forward_offset_mm","left_offset_mm","initial_raise_mm","drop_from_nominal_mm","final_tilt_deg","first_floor_s_after_release"])
        w.writerows(seating)

if __name__=="__main__":
    main()

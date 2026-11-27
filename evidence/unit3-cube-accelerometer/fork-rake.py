"""Pitch the printed fork about its wrist, preserving measured chassis, rail and wrist trajectories."""
import csv
from pathlib import Path
import sys
from xml.etree import ElementTree as ET
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

here=Path(__file__).resolve().parent
repo=Path(sys.argv[2]).resolve() if len(sys.argv)>2 else here.parents[1]
sys.path[:0]=[str(repo),str(repo/"evidence/unit3-printed-fork-contact")]
from replay_links import add_links,rail_paths,require_printed_fork
from simulacrum import params

tree=ET.parse(sys.argv[1])
require_printed_fork(tree)
add_links(tree,repo)
m=mj.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding="unicode"))
v=params.values(params.load(model="vector_2"))
path=np.genfromtxt(here/"pickup-path.csv",names=True,delimiter=",")
sights=np.genfromtxt(here/"pickup-sightings.csv",names=True,delimiter=",")
cube=m.body("cube").id
fork=m.body("fork").mocapid[0]
rails=[m.body(f"rail{i}").mocapid[0] for i in range(2)]
with (here/"fork-rake.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("dt_ms rake_deg slide_cross_s slide_2p59_mm slide_2p86_mm max_rise_mm held_rise_mm".split())
    for dt in (.001,.0005):
        m.opt.timestep=dt
        t=np.arange(-.3,8.31,dt)
        x,y,h,l,p=[np.interp(t,path["time_s"],path[k]) for k in ("x_mm","y_mm","yaw_rad","lift_mm","pitch_rad")]
        bx,_,bz=v["lift_base_position_mm"]
        reach=bx+np.sqrt(v["lift_arm_length_mm"]**2-(l-bz)**2)
        wrist=.001*np.column_stack([x+reach*np.cos(h),y+reach*np.sin(h),l])
        points,quats=rail_paths(repo,x,y,h,l)
        rot=Rotation.from_rotvec(np.column_stack([np.sin(h)*p,-np.cos(h)*p,np.zeros_like(t)]))
        axis=.001*np.column_stack([x,y,np.full_like(t,v["wheel_front_radius_mm"])])
        wrist=axis+rot.apply(wrist-axis)
        points=np.stack([axis+rot.apply(points[:,j]-axis) for j in range(2)],axis=1)
        quats=(rot*Rotation.from_quat(quats,scalar_first=True)).as_quat(scalar_first=True)
        body_rot=rot*Rotation.from_euler("z",h[:, None])
        for rake in (0,-10,-5,5,10):
            orient=(body_rot*Rotation.from_euler("y",rake,degrees=True)).as_quat(scalar_first=True)
            d=mj.MjData(m)
            pose=d.joint("cube_root").qpos
            s=sights[-1]
            yaw=Rotation.from_quat([s[k] for k in ("qx","qy","qz","qw")]).as_euler("xyz")[2]
            pose[:]=[s["x_mm"]/1000,s["y_mm"]/1000,.0222,np.cos(yaw/2),0,0,np.sin(yaw/2)]
            series=[]
            for i,clock in enumerate(t):
                d.mocap_pos[fork]=wrist[i]
                d.mocap_quat[fork]=orient[i]
                d.mocap_pos[rails]=points[i]
                d.mocap_quat[rails]=quats[i]
                mj.mj_step(m,d)
                series.append(pose[:3].copy()*1000)
            xyz=np.array(series)
            rest=np.median(xyz[t<0],axis=0)
            slide=np.linalg.norm(xyz[:,:2]-rest[:2],axis=1)
            rise=xyz[:,2]-rest[2]
            moving=np.flatnonzero((t>=0)&(slide>2))
            w.writerow([dt*1000,rake,t[moving[0]] if moving.size else None,
                        *np.interp([2.5944,2.8593],t,slide),np.max(rise[t>=0]),np.median(rise[t>=6.5])])
            f.flush()

"""Kinematic first fork/cube contact at two sighted poses. Pass forks7 fixture XML."""
import csv
from pathlib import Path
import sys
import mujoco as mj
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
model=mj.MjModel.from_xml_path(sys.argv[1])
fork=model.body("fork").id
cube=model.body("cube").id
assert sum(model.geom_bodyid==fork)>=58
path=np.genfromtxt(here/"descent-input.csv",delimiter=",",names=True)
poses=np.genfromtxt(here/"approach-input.csv",delimiter=",",names=True,dtype=None,encoding="utf8")
fig,(ax,patch)=plt.subplots(1,2,figsize=(10,4),layout="constrained")
with (here/"first-contact.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("sighting forward_shift_mm left_shift_mm first_contact_s wrist_mm cube_geom contact_forward_mm contact_left_mm contact_height_mm".split())
    for dx,dy in [(0,0),(-2,0),(2,0),(0,-2),(0,2)]:
        for name,p in zip(("Penultimate","Final"),poses[poses["run"]=="W20"][-2:]):
            d=mj.MjData(model)
            d.qpos[:3]=[p["cube_x_mm"]*.001,p["cube_y_mm"]*.001,.02215]
            angle=p["robot_yaw"]
            d.qpos[:2]+=.001*np.array([dx*np.cos(angle)-dy*np.sin(angle),dx*np.sin(angle)+dy*np.cos(angle)])
            d.qpos[3:]=[np.cos(p["cube_yaw"]/2),0,0,np.sin(p["cube_yaw"]/2)]
            t=np.arange(path["sec"][0],path["sec"][-1],.001)
            x,y,yaw,pitch,h=[np.interp(t,path["sec"],path[k]) for k in ("x","y","yaw","pitch","lift")]
            # Same rear-axle pivot and linkage as the existing W4 printed-fork replay.
            reach=-41+np.sqrt(66**2-(h-45)**2)
            along=-48+(reach+48)*np.cos(pitch)-(h-10.75)*np.sin(pitch)
            height=10.75+(reach+48)*np.sin(pitch)+(h-10.75)*np.cos(pitch)
            if name=="Penultimate" and dx==dy==0:
                ax.plot(t,h,color=".65",zorder=0)
            for i,sec in enumerate(t):
                d.mocap_pos[0]=.001*np.array([x[i]+along[i]*np.cos(yaw[i]),y[i]+along[i]*np.sin(yaw[i]),height[i]])
                a,b=np.cos(yaw[i]/2),np.sin(yaw[i]/2)
                c,s=np.cos(pitch[i]/2),np.sin(pitch[i]/2)
                d.mocap_quat[0]=[a*c,b*s,-a*s,b*c]
                mj.mj_forward(model,d)
                contacts=[k for k in d.contact if set(model.geom_bodyid[k.geom])=={cube,fork} and k.dist<0]
                if contacts:
                    hit=min(contacts,key=lambda k:k.dist)
                    gid=next(g for g in hit.geom if model.geom_bodyid[g]==cube)
                    off=hit.pos[:2]*1000-[x[i],y[i]]
                    fw=off@[np.cos(yaw[i]),np.sin(yaw[i])]
                    left=off@[-np.sin(yaw[i]),np.cos(yaw[i])]
                    w.writerow([name,dx,dy,sec,h[i],model.geom(gid).name,fw,left,hit.pos[2]*1000])
                    rubber="rubber" in model.geom(gid).name
                    patch.scatter(left,fw,color="C1" if rubber else "C0",marker="o" if name=="Final" else "x")
                    if dx==dy==0:
                        ax.scatter(sec,h[i],label=name+" sighting",s=70 if name=="Final" else 120,
                                   marker="+" if name=="Final" else "o")
                    break
            else:
                w.writerow([name,dx,dy,"","","none","","",""])
ax.set(xlabel="Seconds after wheelstand request",ylabel="Reported wrist height (mm)")
ax.legend()
patch.set(xlabel="Left of chassis origin (mm)",ylabel="Forward (mm)",title="First contact · rubber (orange), shell (blue)")
fig.savefig(here/"first-contact.png",dpi=160)

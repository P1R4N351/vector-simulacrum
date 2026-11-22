"""Replay measured W20 fork motion against a free cube on alternate shelf extents."""
import copy
import csv
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np

here = Path(__file__).resolve().parent
path = np.genfromtxt(here/"descent-input.csv", names=True, delimiter=",")
poses = np.genfromtxt(here/"approach-input.csv", names=True, delimiter=",", dtype=None, encoding="utf8")
tree = ET.parse(sys.argv[1])
chassis = ET.parse(here.parents[2]/"web/public/robot/vector2/vector-contact42-free.xml").find(".//body[@name='chassis']")


def add_front(root, parts):
    body = ET.SubElement(root.find("worldbody"), "body", name="front_contact", mocap="true")
    if "body" in parts:
        body.append(copy.deepcopy(chassis.find("geom[@name='chassis_geom']")))
    if "wheels" in parts:
        for side in ("wheel_fl", "wheel_fr"):
            wheel = chassis.find(f"body[@name='{side}']")
            mount = ET.SubElement(body, "body", pos=wheel.get("pos"))
            mount.append(copy.deepcopy(wheel.find(f"geom[@name='{side}_geom']")))


def replay(pose, clearance, dt, parts=()):
    root = copy.deepcopy(tree.getroot())
    if parts:
        add_front(root, parts)
    table = root.find(".//geom[@name='table']")
    yaw = pose["robot_yaw"]
    forward = np.array([np.cos(yaw), np.sin(yaw)])
    center = np.array([pose["cube_x_mm"], pose["cube_y_mm"]])*.001
    # Project the half width of the upright cube onto the shelf edge normal.
    skew = pose["cube_yaw"]-yaw
    extent = .02215*(abs(np.cos(skew))+abs(np.sin(skew)))
    origin = center + (extent + clearance*.001 - .75)*forward
    table.set("pos",f"{origin[0]} {origin[1]} -0.01")
    table.set("quat",f"{np.cos(yaw/2)} 0 0 {np.sin(yaw/2)}")
    m = mj.MjModel.from_xml_string(ET.tostring(root,encoding="unicode"))
    m.opt.timestep = dt
    d = mj.MjData(m)
    d.qpos[:3] = [*center,.02215]
    d.qpos[3:] = [np.cos(pose["cube_yaw"]/2),0,0,np.sin(pose["cube_yaw"]/2)]
    t = np.arange(4.8,5.85,dt)
    x,y,angle,pitch,h = [np.interp(t,path["sec"],path[k]) for k in ("x","y","yaw","pitch","lift")]
    reach = -41+np.sqrt(66**2-(h-45)**2)
    along = -48+(reach+48)*np.cos(pitch)-(h-10.75)*np.sin(pitch)
    height = 10.75+(reach+48)*np.sin(pitch)+(h-10.75)*np.cos(pitch)
    pos = .001*np.stack([x+along*np.cos(angle), y+along*np.sin(angle), height],axis=1)
    a,b,c,s = np.cos(angle/2),np.sin(angle/2),np.cos(pitch/2),np.sin(pitch/2)
    quat = np.stack([a*c,b*s,-a*s,b*c],axis=1)
    trace=[]
    for i,sec in enumerate(t):
        d.mocap_pos[0],d.mocap_quat[0] = pos[i],quat[i]
        if parts:
            # Rotate chassis origin about the same rear axle as the fork.
            offset = -48 + 48*np.cos(pitch[i]) + 10.75*np.sin(pitch[i])
            z = 10.75 + 48*np.sin(pitch[i]) - 10.75*np.cos(pitch[i])
            d.mocap_pos[1] = .001*np.array([x[i]+offset*np.cos(angle[i]),y[i]+offset*np.sin(angle[i]),z])
            d.mocap_quat[1] = quat[i]
        mj.mj_step(m,d)
        displacement = (d.qpos[:2]-center)*1000
        trace.append((sec+dt,displacement@forward,displacement@[-forward[1],forward[0]],d.qpos[2]*1000))
        if d.qpos[2]<-.02215:
            break
    return np.array(trace)


if __name__ == "__main__":
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout="constrained",sharey=True)
    with (here/"free-departure.csv").open("w") as f:
        wr=csv.writer(f,lineterminator="\n")
        wr.writerow(["pose","clearance_mm","timestep_ms","forward_at_5p4_mm","left_at_5p4_mm","first_center_below_shelf_s"])
        for ax,name,pose in zip(axes,("Penultimate","Final"),poses[poses["run"]=="W20"][-2:]):
            for clr in (-15,-8,0,22,44,500):
                for dt in (.0005,.00025,.000125):
                    trace=replay(pose,clr,dt)
                    below=trace[trace[:,3]<0]
                    wr.writerow([name,clr,dt*1000,*[np.interp(5.4,trace[:,0],trace[:,k]) for k in (1,2)],
                                 below[0,0] if len(below) else ""])
                    if dt==.000125:
                        ax.plot(trace[:,0],trace[:,1],label=f"{clr} mm")
            ax.axvspan(5.03,5.28,alpha=.12,color="grey")
            ax.set(xlabel="Seconds from request",title=name+" cube pose",xlim=(5.1,5.85))
    axes[0].set_ylabel("Forward displacement of cube center (mm)")
    axes[1].legend(title="Initial edge clearance")
    fig.savefig(here/"free-departure.png",dpi=155)

"""Release the articulated model at rest near the two measured held wheelstands."""
import csv
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull

from held_support import here, replay, shell_triangles


def rear_model(v,fixture,points,full_radius=False):
    def edit(tree):
        if full_radius:
            for side in "lr":
                name=f"tip_rear_{side}"
                tree.find(f".//geom[@name='{name}']").set("size",str(v["wheel_rear_radius_mm"]*.001))
                joint=tree.find(f".//equality/joint[@joint1='{name}_joint']")
                joint.set("polycoef",f'0 {v["wheel_front_radius_mm"]/v["wheel_rear_radius_mm"]} 0 0 0')
        table=tree.find(".//geom[@name='table']")
        table.set("conaffinity",str(int(table.get("conaffinity"))|8))
        tree.find("asset").append(ET.Element("mesh",name="rear_surface",vertex=" ".join(
            f"{val*.001:.10g}" for val in points.ravel())))
        body=tree.find(".//freejoint[@name='root']/..")
        attributes=dict(body.find("geom[@name='chassis_geom']").attrib)
        for key in ("size","pos"):
            attributes.pop(key)
        attributes.update(name="rear_surface",mesh="rear_surface",type="mesh",mass="0",contype="8",conaffinity="0")
        ET.SubElement(body,"geom",attributes)
    return replay.build(v,fixture,edit)


def release(m,v,head,height,pitch,points,add_shell):
    d=replay.mj.MjData(m)
    lift=replay.mjcf.lift_angle_of_height(v,height)
    for name,value in (("head",head),("lift",lift),("wrist",-lift)):
        d.joint(name+"_joint").qpos[0]=value
    d.actuator("head").ctrl[0]=head
    d.actuator("lift").ctrl[0]=lift
    d.joint("cube_root").qpos[0]=.4
    theta=np.radians(pitch)
    root=d.joint("root").qpos
    root[3:]=[np.cos(theta/2),0,-np.sin(theta/2),0]
    up=np.array([np.sin(theta),0,np.cos(theta)])
    axle=np.array([-v["wheel_base_mm"],0,v["wheel_rear_radius_mm"]])
    rear_z=m.geom("tip_rear_l").size[0]*1000
    root[2]=(rear_z-axle@up)*.001
    if add_shell:
        root[2]=max(root[2],-(points@up).min()*.001)
    root[2]+=.00005
    rows=[]
    body=m.jnt_bodyid[m.joint("root").id]
    table=m.geom("table").id
    wrench=np.empty(6)
    for i in range(round(1.5/m.opt.timestep)):
        replay.mj.mj_step(m,d)
        if (i+1)%20: continue
        angle=np.degrees(np.arctan2(d.xmat[body,6],d.xmat[body,8]))
        shell_up=0.
        if add_shell:
            for j,c in enumerate(d.contact):
                if set(c.geom)!={table,m.geom("rear_surface").id}: continue
                replay.mj.mj_contactForce(m,d,j,wrench)
                fz=wrench[:3]@c.frame.reshape(3,3)[:,2]
                shell_up+=fz if c.geom[0]==table else -fz
        rows.append([d.time,angle,shell_up])
    return np.array(rows)


def main(fixture,rig):
    v=replay.params.values(replay.params.load(model="vector_2"),{"tip_supports_driven":1})
    xyz=np.unique(shell_triangles(Path(rig),v).reshape(-1,3),axis=0)
    rear=xyz[xyz[:,0]<-v["wheel_base_mm"]]
    rear=rear[ConvexHull(rear).vertices]
    fig,axes=plt.subplots(3,2,figsize=(10,8),layout="constrained",sharex=True,sharey=True)
    rows=[]
    for col,trial in enumerate(("W4","W8")):
        tr=replay.Trial(trial)
        t=tr.state["sec"]-tr.zero-tr.late
        hold=tr.state[(t>=.4)&(t<=.8)]
        target=np.median(np.degrees(hold["pitch"]))
        for row,mode in enumerate(("Box","Shell, inset radius","Shell, full radius")):
            shell=row>0
            m=rear_model(v,fixture,rear,full_radius=row==2) if shell else replay.build(v,fixture)
            m.opt.timestep=.00025
            weight=abs(m.opt.gravity[2])*m.body_subtreemass[m.jnt_bodyid[m.joint("root").id]]
            for offset in (-10,-5,-2,0,2,5,10):
                a=release(m,v,np.median(hold["head"]),np.median(hold["lift"]),target+offset,rear,shell)
                end=a[a[:,0]>1.25]
                rows.append([trial,mode,offset,target,a[-1,1],np.ptp(end[:,1]),np.median(end[:,2])/weight])
                axes[row,col].plot(a[:,0],a[:,1],lw=.9,label=f"{offset:+d}°")
            axes[row,col].axhline(target,color="k",ls="--",lw=.8)
            axes[row,col].set_title(trial+" · "+mode)
    for ax in axes[-1]: ax.set_xlabel("Seconds after release")
    for ax in axes[:,0]: ax.set_ylabel("Chassis pitch (°)")
    axes[0,1].legend(fontsize=7,ncol=3)
    fig.savefig(here/"shell-settle.png",dpi=160)
    with (here/"shell-settle.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("trial geometry start_offset_deg observed_deg final_deg last_quarter_span_deg shell_weight_fraction".split())
        w.writerows(rows)


if __name__=="__main__": main(*sys.argv[1:])

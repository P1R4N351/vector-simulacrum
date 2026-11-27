"""Static support interval when the registered shell meets the shelf on its rear tread."""
import csv
import itertools
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

from chassis_patch import shell_triangles
from acceleration_receipts import here, replay
from scipy.optimize import brentq


def main(fixture, rig):
    v=replay.params.values(replay.params.load(model="vector_2"),{})
    points=np.unique(shell_triangles(Path(rig),v).reshape(-1,3),axis=0)
    model=replay.build(v,fixture)
    box=model.geom("chassis_geom")
    box_points=(box.pos+np.array(list(itertools.product((-1,1),repeat=3)))*box.size)*1000
    axle=np.array([-v["wheel_base_mm"],0,v["wheel_rear_radius_mm"]])
    radius=axle[2]
    def coordinates(vertices,theta):
        delta=vertices-axle
        sn,cs=np.sin(theta),np.cos(theta)
        return delta[:,0]*cs-delta[:,2]*sn, radius+delta[:,0]*sn+delta[:,2]*cs
    theta=brentq(lambda a:coordinates(points,a)[1].min(),0,np.pi/2)
    x,z=coordinates(points,theta)
    heel=x[np.argmin(z)]
    d=replay.mj.MjData(model)
    body=model.jnt_bodyid[model.joint("root").id]
    # Vary the model's level-pose total COM within the Vector 2 parameter bounds.
    # Moving its fixed chassis mass shifts whole-robot COM equally at either joint posture.
    parameters=replay.params.load(model="vector_2")
    rows=[]
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout="constrained")
    for name in ("W4","W8"):
        tr=replay.Trial(name)
        t=tr.state["sec"]-tr.zero-tr.late
        data=[]
        for s,time in zip(tr.state[(t>=0)&(t<=.8)],t[(t>=0)&(t<=.8)]):
            angle=np.arcsin((s["lift"]-v["lift_base_position_mm"][2])/v["lift_arm_length_mm"])
            for key,value in (("lift",angle),("wrist",-angle),("head",s["head"])):
                d.joint(key+"_joint").qpos[0]=value
            replay.mj.mj_forward(model,d)
            com=(d.subtree_com[body]-d.xpos[body])*1000
            cmx=coordinates(com[None,:],theta)[0][0]
            load=cmx/heel
            h=coordinates(points,s["pitch"])[1].min()
            hb=coordinates(box_points,s["pitch"])[1].min()
            bounds=[]
            for ex,ez in itertools.product(("low","high"),repeat=2):
                dx=parameters["parameters"]["com_x_mm"][ex]-v["com_x_mm"]
                dz=parameters["parameters"]["com_z_mm"][ez]-v["com_z_mm"]
                bounds.append(cmx+dx*np.cos(theta)-dz*np.sin(theta))
            rows.append([name,time,np.degrees(s["pitch"]),h,hb,np.degrees(theta),heel,cmx,load,min(bounds),max(bounds)])
            data.append([time,h,hb,cmx])
        data=np.array(data)
        axes[0].plot(data[:,0],data[:,1],".-",label=name+" shell")
        axes[0].plot(data[:,0],data[:,2],".--",label=name+" box")
        axes[1].plot(data[:,0],data[:,3],".-",label=name+" mass centre")
    axes[0].axhline(0,color=".4",lw=.8)
    axes[0].set(ylabel="Lowest point above shelf (mm)",title="Geometry at recorded pitch")
    axes[1].axhspan(heel,0,color=".9",label="Two-contact support interval")
    axes[1].set(ylabel="Forward distance from rear axle (mm)",title="Mass projection at shell touchdown")
    for ax in axes:
        ax.set_xlabel("Seconds from last raised-lift sample")
        ax.legend(fontsize=8)
    fig.savefig(here/"held-support.png",dpi=160)
    with (here/"held-support.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("run seconds pitch_deg rig_clearance_mm box_clearance_mm touchdown_deg heel_x_mm com_x_touchdown_mm shell_weight_fraction com_low_mm com_high_mm".split())
        w.writerows(rows)
    for name in ("W4","W8"):
        a=np.array([r[3:] for r in rows if r[0]==name and r[1]>=.4]);print(name,np.median(a,axis=0))
    print("box touchdown",[(i,coordinates(box_points,np.radians(i))[1].min()) for i in (0,70,90)])


if __name__=="__main__": main(*sys.argv[1:])

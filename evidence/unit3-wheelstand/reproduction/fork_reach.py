"""Bound the printed fork's reach using measured pitch/lift and canonical linkage lengths."""
import csv
import sys
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np

from acceleration_receipts import here, replay


def main(fixture):
    meshes=ET.parse(fixture).findall(".//asset/mesh")
    vertices=np.vstack([np.fromstring(m.get("vertex"),sep=" ").reshape(-1,3)
                        for m in meshes if m.get("name").startswith("lift_fork_geom_stl_")])*1000
    radius=np.hypot(vertices[:,0],vertices[:,2]).max()
    rows=[]
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout="constrained",sharey=True)
    v=replay.params.values(replay.params.load(model="vector_2"),{})
    base=np.asarray(v["lift_base_position_mm"])
    axle=np.array([-v["wheel_base_mm"],0,v["wheel_rear_radius_mm"]])
    for ax,name in zip(axes,("W4","W8")):
        tr=replay.Trial(name)
        t=tr.state["sec"]-tr.zero-tr.late
        sel=(t>=-.18)&(t<=.8)
        series=[]
        for s,time in zip(tr.state[sel],t[sel]):
            pitch=s["pitch"]
            up=np.array([np.sin(pitch),0,np.cos(pitch)])
            # The encoder reports heights below the nominal 32 mm limit; retain them here.
            angle=np.arcsin((s["lift"]-base[2])/v["lift_arm_length_mm"])
            wrist=base+v["lift_arm_length_mm"]*np.array([np.cos(angle),0,np.sin(angle)])
            axle_height=axle[2]-v["tip_support_clearance_mm"]
            wrist_z=axle_height+(wrist-axle)@up
            standard=wrist_z+(vertices@up).min()
            free=wrist_z-radius
            # Even dropping the axle centre all the way to the shelf cannot lower this bound further.
            floor_bound=free-axle_height
            rows.append([name,time,np.degrees(pitch),s["lift"],wrist_z,standard,free,floor_bound])
            series.append([time,standard,free,floor_bound])
        data=np.array(series)
        for i,label in enumerate(("Leveled fork","Any wrist angle","Any wrist angle, axle at shelf"),1):
            ax.plot(data[:,0],data[:,i],".-",label=label)
        ax.axhline(v["cube_outer_mm"],ls="--",color="k",label="Upright cube top")
        ax.set(title=name,xlabel="Seconds from last raised-lift sample")
    axes[0].set_ylabel("Lowest possible fork height (mm)")
    axes[1].legend(fontsize=8)
    fig.savefig(here/"fork-reach.png",dpi=160)
    with (here/"fork-reach.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("run t_s pitch_deg measured_lift_mm wrist_mm leveled_min_mm any_wrist_min_mm axle_at_shelf_min_mm".split())
        w.writerows(rows)
    print("Maximum sagittal radius (mm)",radius)
    for name in ("W4","W8"):
        hold=np.array([r[4:] for r in rows if r[0]==name and r[1]>=.4])
        print(name,hold.min(0),np.median(hold,axis=0),hold.max(0))


if __name__=="__main__": main(sys.argv[1])

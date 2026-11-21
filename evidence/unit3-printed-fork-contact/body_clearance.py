"""Signed distances from the observed trial 1 cube to non-fork robot surfaces."""
import csv
import xml.etree.ElementTree as ET
from replay_links import add_links,rail_paths
import sys
import matplotlib.pyplot as plt
import numpy as np
import mujoco as mj
from front_edge import here, basis, origin, states
from optical_clearance import rows, reconstruct
from evaluate import repo
sys.path.insert(0,str(repo))
from simulacrum import mjcf, params


def main():
    v=params.values(params.load(model="vector_2"))
    xml=ET.ElementTree(ET.fromstring(mjcf.build_mjcf(v,cube="free")))
    add_links(xml,repo)
    m=mj.MjModel.from_xml_string(ET.tostring(xml.getroot(),encoding="unicode"))
    d=mj.MjData(m)
    cubes=np.flatnonzero(m.geom_bodyid==m.body("cube").id)
    groups={"head":["head_geom"],"body":["chassis_geom"],"arm envelope":["lift_arm_geom"],
            "treads":[m.geom(i).name for i in range(m.ngeom)
                      if m.geom(i).name.startswith(("wheel_","tip_front","tip_rear"))]}
    rail_ids=[]
    for i,name in enumerate(("upper links","lower links")):
        body=m.body(f"rail{i}")
        groups[name]=[m.geom(j).name for j in np.flatnonzero(m.geom_bodyid==body.id)]
        rail_ids.append(body.mocapid[0])
    assert groups["treads"]
    r=states[states["trial"]==1]
    fig,axes=plt.subplots(1,2,figsize=(10,4),sharey=True)
    with (here/"body-clearance.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["support","sec","lift_mm","part","signed_gap_mm","robot_geom","cube_geom"])
        for support,ax in zip((False,True),axes):
            series={name:[] for name in groups}
            for t,(rot,delta,_) in zip(rows["sec"],reconstruct(-1.499,473.11,support)):
                if not 3<=t<=7.5:
                    continue
                x,y,yaw,head,lift=[np.interp(t+1.5,r["sec"],r[col]) for col in
                                   ("x_mm","y_mm","yaw_deg","head_rad","lift_mm")]
                half=np.deg2rad(yaw)/2
                d.joint("root").qpos[:]=[x*.001,y*.001,0,np.cos(half),0,0,np.sin(half)]
                angle=mjcf.lift_angle_of_height(v,lift)
                d.joint("lift_joint").qpos[0]=angle
                d.joint("wrist_joint").qpos[0]=-angle
                d.joint("head_joint").qpos[0]=head
                d.joint("cube_root").qpos[:]=np.r_[(origin+basis.apply(delta))*.001,
                                                   (basis*rot).as_quat(scalar_first=True)]
                centers,quats=rail_paths(repo,*[np.array([n]) for n in (x,y,np.deg2rad(yaw),lift)])
                d.mocap_pos[rail_ids]=centers[0]
                d.mocap_quat[rail_ids]=quats[0]
                mj.mj_forward(m,d)
                for part,names in groups.items():
                    gap,robot,cube=min((mj.mj_geomDistance(m,d,m.geom(name).id,c,.25,None)*1000,
                                        name,m.geom(c).name) for name in names for c in cubes)
                    w.writerow([support,t,lift,part,gap,robot,cube])
                    series[part].append((t,gap))
            for part,values in series.items():
                ax.plot(*np.array(values).T,".-",label=part)
            ax.axhline(0,color=".3",lw=.7)
            ax.set(xlabel="Seconds after approach",title="Shelf constrained" if support else "Free pose")
    axes[0].set_ylabel("Closest signed gap (mm)")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(here/"body-clearance.png",dpi=150)


if __name__=="__main__":
    main()

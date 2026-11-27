"""Isolate rubber/table friction, checking explicit contact pairs against native mixing."""
import csv
from unittest.mock import patch
import xml.etree.ElementTree as ET
import sys

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

from ascent_projection import dock_camera, glyph_edges, predicted
from wheel_authority import here, replay, servo


def simulate(v,fixture,tr,mu):
    def pairs(tree):
        contact=tree.find("contact")
        if contact is None:
            contact=ET.SubElement(tree,"contact")
        corners=tree.findall(".//body[@name='cube']/geom")
        for geom in corners:
            name=geom.get("name")
            if name.startswith("cube_rubber_"):
                ET.SubElement(contact,"pair",geom1="table",geom2=name,condim="3",
                              friction=f"{mu} {mu} .005 .0001 .0001",solref=".008 1")
    m=replay.build(v,fixture,edit_tree=pairs if mu is not None else None)
    m.opt.timestep=.00025
    a=m.actuator("lift")
    a.gainprm[0],a.biasprm[1],a.biasprm[2]=5,-5,-.03
    times,positions,rotations=[],[],[]
    sample=replay._Run.sample
    def capture(r,i,t):
        sample(r,i,t)
        times.append(t-tr.late)
        positions.append(r.d.xpos[r.cube_id].copy()*1000)
        rotations.append(r.d.xmat[r.cube_id].reshape(3,3).copy())
    with patch.object(replay._Run,"sample",capture):
        trace,_,score=servo(m,v,tr,3.5,float(m.actuator("left_wheel").gainprm[0]),1.5)
    return trace,score,np.asarray(times),np.array(positions),np.array(rotations)


def main(fixture):
    rows=[]
    v=replay.params.values(replay.params.load(model="vector_2"),{"tip_supports_driven":1})
    tracks=list(csv.DictReader((here/"ascent-cube.csv").open()))
    ts=np.array([float(next(r["seconds"] for r in tracks if int(r["frame"])==i)) for i in range(91,104)])
    def track(key):
        return np.array([np.mean([[float(r[k]) for k in ("dx_px","dy_px")] for r in tracks
                                  if int(r["frame"])==i and r["patch"]==key],axis=0) for i in range(91,104)])
    drift=(track("wall screw")+track("dock outline"))/2
    obs=np.stack([track(key)-drift for key in ("left rim","top rim")],axis=1)
    cam=dock_camera(675,glyph_edges(cv.imread(str(here.parent/"W8/precontact-crop.jpg"),0)))
    fig,axes=plt.subplots(1,3,figsize=(12,4),layout="constrained")
    for ax,name in zip(axes,("W4","W8")):
        tr=replay.Trial(name)
        t=tr.state["sec"]-tr.zero-tr.late
        sel=(t>=-.14)&(t<=.8)
        ax.plot(t[sel],np.degrees(tr.state["pitch"][sel]),"ko-",label="Physical",ms=3)
        for mu in (None,.3,.9,1.5,3.0):
            trace,score,*traj=simulate(v,fixture,tr,mu)
            label="Native 0.9" if mu is None else f"Pair {mu}"
            ax.plot(trace[:,0]-tr.late,trace[:,1],label=label,lw=1)
            scores=[]
            if name=="W8":
                est=predicted(tr,cam,traj,ts,44)
                for window in (slice(None),slice(4,8)):
                    e=(est-obs)[window]
                    scores.append(np.sqrt(np.mean(np.sum((e[1:]-e[0])**2,axis=2))))
                axes[2].plot(ts,est[:,0,0],label=label,lw=1)
            rows.append([name,label,*score[:3],*scores])
        ax.set(xlim=(-.14,.8),title=name,ylabel="Chassis pitch (°)",xlabel="Seconds from raised-lift sample")
    axes[2].plot(ts,obs[:,0,0]-obs[0,0,0],"ko-",ms=3)
    axes[2].set(title="W8 left cube rim",xlabel="Seconds from raised-lift sample",ylabel="Horizontal change (px)")
    axes[0].legend(fontsize=7)
    fig.savefig(here/"corner-grip.png",dpi=160)
    with (here/"corner-grip.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("run mode lift_rmse_mm peak_deg held_deg w8_whole_px w8_ascent_px".split())
        for row in rows: w.writerow(row+[""]*(7-len(row)))


if __name__=="__main__": main(sys.argv[1])

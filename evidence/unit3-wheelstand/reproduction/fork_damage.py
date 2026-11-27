"""Bracket asymmetric fork-tip loss by clipping one printed upright at a time."""
import csv
import sys
from unittest.mock import patch

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull

from corner_grip import dock_camera, glyph_edges, here, predicted, replay, simulate


def shorten(tree,side,depth):
    for mesh in tree.findall(".//asset/mesh"):
        vertices=np.fromstring(mesh.get("vertex"),sep=" ").reshape(-1,3)
        if (side*vertices[:,1]).min()<.014: continue
        z=(2.4-depth)*.001
        keep=vertices[:,2]<=z
        if keep.all(): continue
        if not keep.any():
            body=tree.find(".//body[@name='lift_fork']")
            body.remove(body.find(f"geom[@mesh='{mesh.get('name')}']"))
            continue
        pts=list(vertices[keep])
        for face in ConvexHull(vertices).simplices:
            for a,b in zip(face,np.roll(face,1)):
                if keep[a]!=keep[b]:
                    p,q=vertices[[a,b]]
                    pts.append(p+(q-p)*(z-p[2])/(q[2]-p[2]))
        pts=np.unique(np.round(pts,10),axis=0)
        mesh.set("vertex"," ".join(f"{x:.10g}" for x in pts.ravel()))
        mesh.attrib.pop("face",None)


def main(fixture):
    v=replay.params.values(replay.params.load(model="vector_2"),{"tip_supports_driven":1})
    data=list(csv.DictReader((here/"ascent-cube.csv").open()))
    frames=list(range(91,104))
    times=np.array([float(next(r["seconds"] for r in data if int(r["frame"])==i)) for i in frames])
    def path(key):
        return np.array([np.mean([[float(r[k]) for k in ("dx_px","dy_px")] for r in data
                                  if int(r["frame"])==i and r["patch"]==key],axis=0) for i in frames])
    obs=np.stack([path(k)-(path("wall screw")+path("dock outline"))/2 for k in ("left rim","top rim")],axis=1)
    cam=dock_camera(675,glyph_edges(cv.imread(str(here.parent/"W8/precontact-crop.jpg"),0)))
    base=replay.build
    sample=replay._Run.sample
    fig,axes=plt.subplots(1,3,figsize=(12,4),layout="constrained")
    rows=[]
    for ax,name in zip(axes,("W4","W8")):
        tr=replay.Trial(name)
        t=tr.state["sec"]-tr.zero-tr.late
        mask=(t>=-.14)&(t<=.8)
        ax.plot(t[mask],np.degrees(tr.state["pitch"][mask]),"ko-",ms=3,label="Physical")
        for side,depth in ((0,0),(1,2),(-1,2),(1,8),(-1,8),(1,24),(-1,24)):
            def build(v,fx,edit_tree=None):
                assert edit_tree is None
                return base(v,fx,edit_tree=(lambda tree:shorten(tree,side,depth)) if depth else None)
            impulse=np.zeros((2,2))
            patches=set()
            heights=[]
            def inspect(r,i,t):
                sample(r,i,t)
                if not tr.late<=t<=tr.late+.275: return
                fork=r.m.body("lift_fork").id
                for j,c in enumerate(r.d.contact):
                    if set(r.m.geom_bodyid[c.geom])!={fork,r.cube_id}: continue
                    f=np.empty(6)
                    replay.mj.mj_contactForce(r.m,r.d,j,f)
                    xyz=(c.pos-r.d.xpos[fork])@r.d.xmat[fork].reshape(3,3)
                    if f[0]>0:
                        heights.append(xyz[2]*1000)
                        patches.add(r.m.geom(next(g for g in c.geom if r.m.geom_bodyid[g]==fork)).name)
                    impulse[int(xyz[1]>0),int(xyz[2]>-.0056)] += f[0]*r.dt*4
            with patch.object(replay,"build",build), patch.object(replay._Run,"sample",inspect):
                trace,score,*traj=simulate(v,fixture,tr,None)
            label="Intact" if not side else f"{'Left' if side>0 else 'Right'} −{depth} mm"
            ax.plot(trace[:,0]-tr.late,trace[:,1],label=label,lw=1)
            rmse=[]
            if name=="W8":
                p=predicted(tr,cam,traj,times,44)
                for sel in (slice(None),slice(4,8)):
                    e=(p-obs)[sel]; rmse.append(np.sqrt(np.mean(np.sum((e[1:]-e[0])**2,axis=2))))
                axes[2].plot(times,p[:,0,0],lw=1)
            rows.append([name,label,*score[:3],*impulse.ravel(),min(heights),max(heights),",".join(sorted(patches)),*rmse])
        ax.set(xlim=(-.14,.8),xlabel="Seconds from last raised-lift sample",ylabel="Pitch (°)",title=name)
    axes[2].plot(times,obs[:,0,0]-obs[0,0,0],"ko-",ms=3)
    axes[2].set(xlabel="Seconds from last raised-lift sample",ylabel="Horizontal displacement (px)",title="W8 left cube rim")
    axes[0].legend(fontsize=7)
    fig.savefig(here/"fork-damage.png",dpi=160)
    with (here/"fork-damage.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("run trim lift_rms_mm peak_deg held_deg right_lower_Ns right_tip_Ns left_lower_Ns left_tip_Ns min_z_mm max_z_mm pieces full_px ascent_px".split())
        for row in rows: w.writerow(row+[""]*(14-len(row)))


if __name__=="__main__": main(sys.argv[1])

"""Estimate a timing shift from one fork tip and test against the other."""
import csv
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import cv2 as cv
import numpy as np
from dock_outline import glyph_edges
from fork_projection import EARLY,FORK,PICKS,dock_camera,intersect,project_fork,rot,LENS_X_MM
import replay

HERE=Path(__file__).resolve().parent
MOVING=(64,66,68)
SIDES=("near","far")


def main(fixture,crop):
    tr=replay.Trial("W8")
    v=replay.params.values(replay.params.load(model="vector_2"),{})
    xyz=np.vstack([np.fromstring(m.get("vertex"),sep=" ").reshape(-1,3)*1000
                   for m in ET.parse(fixture).findall(".//asset/mesh") if m.get("name","").startswith("lift_fork_geom_stl_")])
    xyz=xyz[(abs(xyz[:,1])>17)&(xyz[:,2]>0)]
    cam=dock_camera(675,glyph_edges(cv.imread(crop,0).astype(float)))
    a,b=intersect(FORK,93,cam)
    yaw0=np.arctan2(b[0]-a[0],a[1]-b[1])
    lens86=intersect(np.array([[790.,683.]]),65.5,cam)[0]
    if (intersect(np.array([[732.,651.]]),65.5,cam)[0]-lens86)@[np.cos(yaw0),np.sin(yaw0)]<0:
        yaw0+=np.pi
    base=yaw0-tr.obs(3.065,"yaw")
    pos=lambda t:np.array([tr.obs(t,k) for k in ("x_mm","y_mm")])
    axle=intersect(EARLY[1:]*2,65.5,cam)[0]-rot(base+tr.obs(.625,"yaw"))@np.array([LENS_X_MM,0])
    times={int(r["frame"]):r["approx_s"] for r in np.genfromtxt(HERE/"fork-projection-dock-edges.csv",names=True,delimiter=",")}
    def tips(frame,lag):
        t=times[frame]
        h=base+tr.obs(t,"yaw")
        lens=axle+rot(base)@(pos(t)-pos(.625))+rot(h)@np.array([LENS_X_MM,0])
        uv=project_fork(v,tr,t,cam,h,lens,xyz,lift_time=t+lag)
        peaks=[]
        for side in (-1,1):
            s=uv[side*xyz[:,1]>17]
            peaks.append(s[s[:,1]<s[:,1].min()+1].mean(axis=0))
        return np.array(sorted(peaks,key=lambda q:q[0]))
    shifts=np.arange(-80,81)*.001
    errors=[]
    for dt in shifts:
        # Stationary reference is fixed at its measured time under every candidate shift.
        ref=tips(74,0)-np.array(PICKS[74])
        errors.append([tips(k,dt)-np.array(PICKS[k])-ref for k in MOVING])
    err=np.array(errors)
    rmse=lambda x: np.sqrt(np.mean(np.sum(x*x,axis=-1)))
    fig,panels=plt.subplots(1,2,figsize=(10,4),layout="constrained",sharey=True)
    with (HERE/"tip-phase.csv").open("w",newline="") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["fit_side","score_side","held_out_frame","advance_ms","baseline_rms_px","shifted_rms_px"])
        zero=int(np.argmin(abs(shifts)))
        for side in (0,1):
            other=1-side
            for held in (None,*range(len(MOVING))):
                train=[i for i in range(len(MOVING)) if i!=held]
                test=train if held is None else [held]
                idx=np.argmin([rmse(e[train,side]) for e in err])
                if held is None:
                    observed=np.array([np.array(PICKS[k])[other]-np.array(PICKS[74])[other] for k in MOVING])
                    ts=[times[k] for k in MOVING]
                    ax=panels[other]
                    ax.plot(ts,observed[:,1],"ko-",label="Camera picks")
                    ax.plot(ts,observed[:,1]+err[zero,:,other,1],"s--",label="Recorded lift at frame time")
                    ax.plot(ts,observed[:,1]+err[idx,:,other,1],"^--",label=f"Advance from {SIDES[side]} tip ({shifts[idx]*1000:.0f} ms)")
                    ax.set_title("Held-out "+SIDES[other]+" upright")
                    ax.set_xlabel("Seconds after command")
                    ax.legend(fontsize=8)
                w.writerow([SIDES[side],SIDES[other],MOVING[held] if held is not None else "all",round(shifts[idx]*1000),
                            rmse(err[zero,test,other]),rmse(err[idx,test,other])])

    panels[0].set_ylabel("Vertical position below plateau reference (px)")
    fig.savefig(HERE/"tip-phase.png",dpi=160)


if __name__=="__main__":
    main(*sys.argv[1:])

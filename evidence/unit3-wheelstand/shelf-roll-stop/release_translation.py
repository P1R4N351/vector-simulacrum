"""Estimate camera-frame cube center displacement across the shelf release."""
import csv
from pathlib import Path
import cv2
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
marker=np.array([[-1.,-1,0],[1,-1,0],[1,1,0],[-1,1,0]])*11.5
corners=np.loadtxt(here/"landing-corners.csv",delimiter=",",skiprows=1)

def pose(q,k,branch=0):
    _,rs,ts,errors=cv2.solvePnPGeneric(marker,q.reshape(4,2),k,None,flags=cv2.SOLVEPNP_IPPE)
    rot=cv2.Rodrigues(rs[branch])[0]
    return ts[branch].ravel()+22*rot[:,2],rot,errors[branch].item()

def main():
    rows=[]
    fig,ax=plt.subplots(figsize=(8,4),layout="constrained")
    for th in (80,90,100):
        data=corners[corners[:,1]==th]
        for focal in (.95,1.,1.05):
            k=np.array([[465*focal,0,329.5],[0,465*focal,70.5],[0,0,1.]])
            for ref,idx in (("before",data[:,0]<0),("after",data[:,0]>6)):
                _,base,_=pose(np.median(data[idx,2:],axis=0),k)
                # Upright glyph horizontal and face normal span the shelf plane.
                for branch in (0,1):
                    track=[]
                    for t,_,*q in data:
                        if not 4.35<t<5.75: continue
                        center,_,err=pose(np.array(q),k,branch)
                        track.append([t,*(center@base),err])
                    a=np.asarray(track)
                    held=(a[:,0]>5.15)&(a[:,0]<5.23)
                    rest=(a[:,0]>5.33)&(a[:,0]<5.45)
                    assert held.sum()>=2 and rest.sum()>=2
                    start=np.median(a[held,1:4],axis=0)
                    delta=np.median(a[rest,1:4],axis=0)-start
                    dt=np.median(a[rest,0])-np.median(a[held,0])
                    spread=max(np.ptp(a[held,1]),np.ptp(a[rest,1]))
                    # Drop one frame from either endpoint to expose sensitivity to individual corners.
                    omitted=[]
                    for group in (held,rest):
                        for j in np.flatnonzero(group):
                            mask=np.arange(len(a))!=j
                            shift=np.median(a[rest&mask,1:4],axis=0)-np.median(a[held&mask,1:4],axis=0)
                            omitted.append(np.hypot(shift[0],shift[2]))
                    rows.append([th,focal,ref,branch,held.sum(),rest.sum(),dt,*delta,
                                 np.hypot(delta[0],delta[2]),spread,np.median(a[held,-1]),np.median(a[rest,-1]),min(omitted),max(omitted)])
                    if focal==1 and ref=="before" and branch==0:
                        ax.plot(a[:,0],np.where(a[:,-1]<=.4,a[:,1]-start[0],np.nan),".-",label=f"Threshold {th}")
    ax.axvspan(5.15,5.23,color=".85",zorder=-1)
    ax.axvspan(5.33,5.45,color=".85",zorder=-1)
    ax.set(xlim=(5.05,5.55),xlabel="Seconds from roll command",ylabel="Center along resting glyph horizontal (mm)")
    ax.legend()
    fig.savefig(here/"release-translation.png",dpi=155)
    with (here/"release-translation.csv").open("w") as stream:
        w=csv.writer(stream,lineterminator="\n")
        w.writerow("threshold focal_multiplier reference branch held_frames post_frames median_dt_s transverse_mm down_mm depth_mm shelf_norm_mm within_window_transverse_range_mm held_error_px post_error_px omit_one_min_mm omit_one_max_mm".split())
        w.writerows(rows)
    for b in [0,1]:
        group=np.array([[r[i] for i in [7,10,11,12,13]] for r in rows if r[3]==b])
        print(b,group.min(axis=0),group.max(axis=0))

if __name__=="__main__":main()

"""Compare change in fork approach heading relative to cube from a transverse shell seam."""
import csv
from pathlib import Path

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
observations=list(csv.DictReader((here/"shelf-slide.csv").open()))
glyph=np.float32([[r[k].split() for k in r if k.startswith("glyph_")] for r in observations])*np.float32(2/3)
square=np.float32([[0,0],[1,0],[1,1],[0,1]])
path=np.genfromtxt(here/"pickup-path.csv",names=True,delimiter=",")
sim=np.genfromtxt(here/"shelf-yaw-sim.csv",names=True,delimiter=",",dtype=None,encoding="utf8")
sim=sim[sim["initialization"]=="Last sighting + pitch"]
cap=cv.VideoCapture(str(here/"pickup.mp4"))
# Central straight portion of dark shell seam. Avoid both rolled ends.
samples=[(0,16,(239,135,263,142)),(4,24,(228,147,251,155)),
         (5,28,(201,166,223,178)),(6,30,(194,170,215,183))]
def traces(gray,seed,halfwidth):
    x0,y0,x1,y1=seed
    xs=np.arange(x0,x1+1)
    ys=[]
    for x in xs:
        mid=round(y0+(x-x0)*(y1-y0)/(x1-x0))
        slab=gray[mid-halfwidth:mid+halfwidth+1,x].astype(float)
        weights=slab.max()-slab
        ys.append(mid-halfwidth+np.average(np.arange(len(slab)),weights=weights))
    return np.float32(np.column_stack([xs,ys]))
def angle(v):
    return -np.degrees(np.arctan2(v[1],v[0]))
def direction(points):
    return angle(np.linalg.lstsq(np.column_stack([np.arange(len(points)),np.ones(len(points))]),points,rcond=None)[0][0])
def rectify(points,H):
    return cv.perspectiveTransform(points[None],H)[0]
def near(a,b):
    return (a-b+180)%360-180
results=[]
fig,axs=plt.subplots(2,2,figsize=(9,7),layout='constrained')
for cidx in (0,1,2,3,4):
    H=cv.getPerspectiveTransform(glyph[cidx],square)
    for halfwidth,trim in [(h,k) for h in (4,5,6) for k in (0,3)]:
        for idx,frame,seed in samples:
            t=float(observations[idx]['time_s'])
            cap.set(cv.CAP_PROP_POS_FRAMES,frame)
            ok,im=cap.read()
            assert ok
            picks=traces(cv.cvtColor(im,cv.COLOR_BGR2GRAY),seed,halfwidth)
            keep=picks[trim:len(picks)-trim]
            seam=direction(rectify(keep,H))
            quad=rectify(glyph[idx],H)
            edges=np.roll(quad,-1,axis=0)-quad
            cube=np.mean([near(angle(v),a) for v,a in zip(edges,(0,-90,-180,90))])
            odom=np.degrees(np.interp(t,path["time_s"],np.unwrap(path["yaw_rad"])))
            cube_sim=np.interp(t,sim["time_s"],sim["yaw_change_deg"])
            if idx==0:
                zero=np.array([seam,cube,odom,cube_sim])
            changes=np.array([seam,cube,odom,cube_sim])-zero
            results.append([t,cidx,trim,halfwidth,*changes,changes[0]-changes[1],changes[2]-changes[3]])
            if cidx==0 and trim==0 and halfwidth==5:
                ax=axs.ravel()[[r[0] for r in samples].index(idx)]
                ax.imshow(cv.cvtColor(im,cv.COLOR_BGR2RGB))
                ax.plot(picks[:,0],picks[:,1],color='cyan',lw=1.2)
                ring=np.vstack([glyph[idx],glyph[idx,0]])
                ax.plot(*ring.T,color='gold',lw=1)
                ax.set(xlim=(30,325),ylim=(340,105),title=f"+{t:.2f} s")
                ax.axis('off')
cap.release()
with (here/"approach-heading.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("time_s rectification_frame trim_px halfwidth_px shell_yaw_change_deg cube_yaw_change_deg sdk_yaw_change_deg sim_cube_yaw_change_deg relative_camera_change_deg relative_replay_change_deg".split())
    w.writerows(results)
fig.savefig(here/"approach-heading.png",dpi=145)
for t in sorted(set(r[0] for r in results)):
    a=np.array([r[4:] for r in results if r[0]==t])
    print(t,a.min(axis=0).round(2),a.max(axis=0).round(2))
fig,axes=plt.subplots(1,2,figsize=(9,3.8),layout='constrained')
a=np.asarray(results)
ts=np.unique(a[:,0])
for ax,cam,pred,title in [(axes[0],4,6,"Robot heading change"),(axes[1],8,9,"Robot minus cube heading change")]:
    readings=[a[a[:,0]==t,cam] for t in ts]
    lo,mid,hi=[[fn(v) for v in readings] for fn in (np.min,np.median,np.max)]
    ax.fill_between(ts,lo,hi,alpha=.2,color="tab:blue",label="Image processing range")
    ax.plot(ts,mid,"o-",label="Camera median")
    ax.plot(ts,[a[a[:,0]==t,pred][0] for t in ts],"x--",label="Recorded-path replay")
    ax.set(title=title,xlabel="Seconds from pickup command",ylabel="Degrees (CCW positive)")
axes[0].legend(fontsize=8)
fig.savefig(here/"approach-heading-overlay.png",dpi=150)

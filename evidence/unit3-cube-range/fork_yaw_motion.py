"""Track two bilateral fork features over the final second of five physical dock returns."""
import csv
import io
import json
import subprocess
import zipfile
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

from fork_heading import centroid, rays, read, references, seeds, yaw
from recorded_mount import ROOT, read_trial, states, table


def track(anchor, image, points):
    args = dict(winSize=(19,19), maxLevel=3)
    p = np.float32(points).reshape(-1,1,2)
    q, ok, _ = cv2.calcOpticalFlowPyrLK(anchor, image, p, None, **args)
    back, ok2, _ = cv2.calcOpticalFlowPyrLK(image, anchor, q, None, **args)
    if not (ok.all() and ok2.all()):
        return None
    error = np.linalg.norm(back-p,axis=2).max()
    def patch(im, point):
        return cv2.getRectSubPix(im,(13,13),tuple(point.ravel()))
    corr = min(cv2.matchTemplate(patch(anchor,a),patch(image,b),cv2.TM_CCOEFF_NORMED).item()
               for a,b in zip(p,q))
    if error > 1 or corr < .80:
        return None
    return q[:,0].astype(float),error,corr


def main():
    contacts = table("return-scores.csv")
    trials = [read_trial(c,o)[0] for c,o in zip(contacts,table("mount-target-inputs.csv"),strict=True)]
    raw=subprocess.check_output(["node",str(ROOT/"recorded_mount.mjs")],input=json.dumps(trials),text=True)
    sim=np.genfromtxt(io.StringIO(raw),delimiter=",",names=True,dtype=None,encoding="utf8")
    fig,axes=plt.subplots(2,5,figsize=(14,6),layout="constrained")
    result=[]
    with zipfile.ZipFile(ROOT/"return-optics.zip") as images, zipfile.ZipFile(ROOT/"release-optics.zip") as dock:
        for col, contact in enumerate(contacts):
            name=contact["session"]
            frames=sorted((float(Path(k).stem),k) for k in images.namelist() if k.startswith(name+"/-"))
            ref=read(images,frames[-1][1])
            points=np.array([centroid(ref,p) for p in seeds[name]])
            _,calib=references(name,read(dock,name+"/dock.png"))
            rows,_=states(name)
            t=np.array([float(r["receipt_s"])-contact["first_on_s"] for r in rows])
            use=(t>=-2)&(t<=0)
            t=t[use]
            rows=[r for r,keep in zip(rows,use) if keep]
            rate=np.array([float(r["gz"])/np.cos(float(r["head_rad"])) for r in rows])
            lift=np.array([float(r["lift_mm"]) for r in rows])
            gyro=np.rad2deg(np.r_[0,np.cumsum((rate[:-1]+rate[1:])*.5*np.diff(t))])
            records=[]
            geometry=[]
            for at,k in frames:
                img=read(images,k)
                tracked=track(ref,img,points)
                if tracked is None or np.ptp(lift[(t>=at-.08)&(t<=frames[-1][0]+.08)])>.11:
                    continue
                uv,err,corr=tracked
                angles=[]
                variants=[]
                for p,h in [(uv[:2],50),(uv[2:],40)]:
                    origin,dirs=rays(calib,473.11,p)
                    angles.append(yaw(origin,dirs,h,0))
                    variants.append([yaw(origin,dirs,h+dh,dz) for dh in [-3,3] for dz in [-.5,.5]])
                geometry.append(variants)
                if not records:
                    ax=axes[0,col]
                    ax.imshow(img,cmap="gray")
                    for pair in [uv[:2],uv[2:]]:
                        ax.plot(*pair.T,"+-",lw=.8)
                    ax.set(xlim=(35,210),ylim=(245,100),title=f"{name} · {at:+.2f} s")
                    ax.axis("off")
                records.append([at,*angles,err,corr])
            a=np.array(records)
            geom=np.array(geometry)
            # Allow the earlier and later feature heights and cross-fork tilt to vary independently.
            low=(geom.min(axis=2)-geom[0].max(axis=1)).min(axis=1)
            high=(geom.max(axis=2)-geom[0].min(axis=1)).max(axis=1)
            low[0] = high[0] = 0.0
            at=a[:,0]
            a[:,1:3]-=a[0,1:3].copy()
            g=np.interp(at,t,gyro)-np.interp(at[0],t,gyro)
            ax=axes[1,col]
            ax.fill_between(at,low,high,color="gray",alpha=.18,label="Height sensitivity")
            for v,label in [(a[:,1],"Fork tips"),(a[:,2],"Lower pair"),(g,"Head gyro")]:
                ax.plot(at,v,".-",label=label)
            preds=[]
            for variant,label in [("canonical","Canonical"),("driven_rear_envelope","Driven support")]:
                run=sim[(sim["session"]==name)&(sim["variant"]==variant)]
                pred=np.interp(at,run["t_s"],run["yaw_deg"])
                pred-=pred[0]
                preds.append(pred)
                ax.plot(at,pred,"--",label=label)
            result.extend([[name,*row,gv,p1,p2,lo,hi] for row,gv,p1,p2,lo,hi in zip(a,g,*preds,low,high)])
            ax.set(xlabel="Time from first on-report (s)",ylim=(-24,14))
    axes[1,0].legend(fontsize=7)
    axes[1,0].set_ylabel("Change from first retained image (°)")
    fig.savefig(ROOT/"fork-yaw-motion.png",dpi=160)
    with (ROOT/"fork-yaw-motion.csv").open("w") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow("session t_s tip_deg lower_deg fb_error_px min_corr gyro_deg canonical_deg driven_deg low_deg high_deg".split())
        writer.writerows(result)


if __name__=="__main__":
    main()

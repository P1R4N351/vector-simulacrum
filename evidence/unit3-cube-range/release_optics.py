"""Track fork tips in rectified native thumbnails; optional RAW directory generates the archive."""
import csv
import gzip
import json
import sys
import zipfile
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
archive = here/"release-optics.zip"
# Working crop in the previously rotated native image, after lens correction.
offset = np.array([705., 510.])
principal = np.array([912.9463,541.1996]) - offset
world = np.array([[-23,0,71.5],[23,0,71.5],[23,0,25.5],[-23,0,25.5],[0,-94,0]],float)
# Outer glyph corners extrapolated over the rounded corners; midpoint of dock front lip.
cal = {"A":(105,60,177,129,141,248),"B":(102,48,175,116,138,233),
       "C":(102,48,175,116,138,233),"D":(102,48,175,116,138,233),
       "E":(104,51,177,119,140,237),"F":(104,51,177,119,140,237)}
tip_y = dict(zip("ABCDEF",(174,163,162,163,165,165)))
def states(name):
    with gzip.open(here.parent/f"unit3-physical-repeats/{name}/robot.csv.gz","rt") as f:
        s=list(csv.DictReader(f))
    i=next(i for i in range(1,len(s)) if int(s[i-1]["status"])&4096 and not int(s[i]["status"])&4096
           and any(float(s[i-1][k]) for k in ("left_mmps","right_mmps")))
    return s,i

def extract(raw):
    files=sorted((raw/"frames").glob("*.jpg"))
    stamps=np.array([float(p.stem) for p in files])
    yy,xx=np.indices((264,270),dtype=np.float32)
    x,y=xx-principal[0],yy-principal[1]
    scale=2/(1+np.sqrt(1+4*1.499*(x*x+y*y)/960**2))
    maps=[a.astype("f") for a in (x*scale+912.9463-420,y*scale+541.1996-485)]
    with zipfile.ZipFile(archive,"w") as z:
        for name in cal:
            stem="physical" if name=="A" else name
            lines=(raw/f"cube-repeat-{stem}.jsonl").read_text().splitlines()
            epoch=next(json.JSONDecoder().raw_decode(l)[0]["t"] for l in lines if '"action": "undock"' in l and '"begin"' in l)
            s,i=states(name)
            times=stamps-epoch-float(s[i]["receipt_s"])
            ids=np.flatnonzero((times>=-1.15)&(times<=.55))
            for j in [*ids, np.argmin(abs(stamps-epoch-30))]:
                gray=cv2.imread(str(files[j]),cv2.IMREAD_GRAYSCALE)
                rect=cv2.remap(gray,*maps,cv2.INTER_LINEAR)
                rect=cv2.GaussianBlur(rect,(3,3),.5)
                filename=f"{times[j]:.5f}" if j in ids else "dock"
                z.writestr(f"{name}/{filename}.png",cv2.imencode(".png",rect)[1].tobytes())

def displacement(ref, im, xy):
    x,y=xy
    patch=ref[y-8:y+9,x-11:x+12]
    area=im[y-26:y+49,x-24:x+25]
    scores=cv2.matchTemplate(area,patch,cv2.TM_CCOEFF_NORMED)
    _,peak,_,(u,v)=cv2.minMaxLoc(scores)
    def refine(a,b,c):
        return .5*(a-c)/(a-2*b+c) if a-2*b+c else 0
    if not (0<u<scores.shape[1]-1 and 0<v<scores.shape[0]-1):return np.array([np.nan]*2),peak
    sub=[refine(*scores[v,u-1:u+2]),refine(*scores[v-1:v+2,u])]
    return np.array([u-13,v-18])+sub,peak

def scene(name):
    x,y,r,b,c,lip=cal[name]
    pts=np.array([[x,y],[r,y],[r,b],[x,b],[c,lip]],float)
    K=np.array([[473.11,0,principal[0]],[0,473.11,principal[1]],[0,0,1.]])
    ok,rot,trans=cv2.solvePnP(world,pts,K,None,flags=cv2.SOLVEPNP_SQPNP)
    assert ok
    rot,trans=cv2.solvePnPRefineLM(world,pts,K,None,rot,trans)
    R=cv2.Rodrigues(rot)[0];origin=-R.T@trans.ravel()
    def ray(uv):
        return R.T@np.linalg.solve(K,np.r_[uv,1])
    def travel(uv, shift, height, slope):
        a=ray(uv);p=origin+a*(height-origin[2])/a[2]
        b=ray(uv+shift)
        # Shelf-forward = -world Y, and Z decreases as the robot descends the ramp.
        n=np.array([0.,-slope,1.])
        q=origin+b*np.dot(n,p-origin)/np.dot(n,b)
        return p[1]-q[1]
    return travel

def main():
    fig,axes=plt.subplots(2,3,figsize=(11,6),layout="constrained")
    replay=np.genfromtxt(here/"release-replay.csv",names=True,delimiter=",")
    rows=[]
    frames=[]
    audit,panels=plt.subplots(2,3,figsize=(9,6),layout="constrained")
    with zipfile.ZipFile(archive) as z:
        for name,ax in zip(cal,axes.flat):
            read=lambda key:cv2.imdecode(np.frombuffer(z.read(key),np.uint8),0)
            keys=sorted((float(Path(k).stem),k) for k in z.namelist() if k.startswith(name+"/") and not k.endswith("dock.png"))
            t=np.array([a for a,k in keys]);imgs=[read(k) for a,k in keys]
            s,i=states(name);st=np.array([float(r["receipt_s"]) for r in s]);st-=st[i]
            lift=np.array([float(r["lift_mm"]) for r in s])
            j=i
            while j>0 and abs(lift[j-1]-lift[i])<.1:j-=1
            base=next(n for n,at in enumerate(t) if at>=st[j])
            patches=[(89,tip_y[name]),(163,tip_y[name])]
            refs=[(45,107 if name!="A" else 118),(230,110 if name!="A" else 121)]
            panel=panels.flat[list(cal).index(name)]
            panel.imshow(imgs[base],cmap="gray")
            for x,y in patches+refs:
                panel.add_patch(plt.Rectangle((x-11,y-8),23,17,fill=False,
                                               edgecolor="lime" if (x,y) in patches else "orange"))
            panel.axis("off");panel.set_title("Session "+name)
            travel=scene(name); est=[]
            for n in range(base,len(t)):
                fixed=[displacement(imgs[base],imgs[n],p)[0] for p in refs]
                drift=np.mean(fixed,axis=0)
                tips=[displacement(imgs[base],imgs[n],p) for p in patches]
                if min(v for _,v in tips)<.8 or not np.all(np.isfinite([d for d,_ in tips])): break
                if any(abs(float(r["lift_mm"])-lift[i])>.1 for r,at in zip(s,st) if t[base]<=at<=t[n]):break
                ds=[travel(p,d-drift,h,slope) for p,(d,_) in zip(patches,tips)
                    for h in (33,43) for slope in (0,.17)]
                est.append((t[n],np.mean(ds),min(ds),max(ds)))
                frames.append([name,*est[-1],*drift,min(v for _,v in tips)])
            e=np.array(est);ax.fill_between(e[:,0],e[:,2],e[:,3],alpha=.2)
            ax.plot(e[:,0],e[:,1],"o-",ms=4)
            ax.axvspan(st[i-1],0,color="gray",alpha=.25)
            ax.vlines(st[i-1]/2, replay["last_on_mm"].min(), replay["first_off_mm"].max(),
                      linewidth=5,color="#bd5225",label="Sim release, 25–60 mm/s")
            if name=="A":ax.legend(fontsize=7)
            ax.set(title=f"Session {name}",xlabel="Seconds from first off-contact report",ylabel="Forward travel (mm)")
            in_bracket=e[(e[:,0]>=st[i-1]) & (e[:,0]<=0)]
            nearest=in_bracket[-1] if len(in_bracket) else [np.nan]*4
            mid=st[i-1]/2
            estimate=np.interp(mid,e[:,0],e[:,1]) if e[0,0]<=mid<=e[-1,0] else np.nan
            rows.append([name,t[base],st[i-1],*nearest,estimate])
    audit.savefig(here/"release-landmarks.png",dpi=150)
    with (here/"release-frames.csv").open("w",newline="") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["session","frame_s","forward_mm","min_mm","max_mm","ref_dx_px","ref_dy_px","min_tip_correlation"])
        w.writerows(frames)
    fig.savefig(here/"release-optics.png",dpi=160)
    with (here/"release-optics.csv").open("w",newline="") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["session","reference_s","last_on_s","in_bracket_s","in_bracket_mm",
                    "in_bracket_min","in_bracket_max","interpolated_midpoint_mm"])
        w.writerows(rows)
if __name__=="__main__":
    if len(sys.argv)>1: extract(Path(sys.argv[1]))
    main()

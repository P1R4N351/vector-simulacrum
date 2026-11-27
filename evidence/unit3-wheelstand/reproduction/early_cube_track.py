"""Track two rigid cube corners against stationary scene features during the W8 lift rise."""
import csv
from pathlib import Path
import sys

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

import replay

HERE = Path(__file__).resolve().parent
PATCHES = {"cube near":(307,347),"cube far":(323,330),"dock glyph":(410,280),"wall screw":(377,287)}
TEMPLATE = 6
SEARCH = 8


def match(frame, reference, xy):
    x,y = xy
    pad = TEMPLATE+SEARCH
    roi = frame[y-pad:y+pad+1,x-pad:x+pad+1]
    template = reference[y-TEMPLATE:y+TEMPLATE+1,x-TEMPLATE:x+TEMPLATE+1]
    corr = cv.matchTemplate(roi,template,cv.TM_CCOEFF_NORMED)
    _,peak,_,(i,j) = cv.minMaxLoc(corr)
    if min(i,j)<1 or max(i,j)>=2*SEARCH:
        return np.full(2,np.nan),peak
    def refine(l,c,r):
        curvature = l-2*c+r
        return .5*(l-r)/curvature if curvature < -1e-8 else 0.
    dx = i-SEARCH+refine(*corr[j,i-1:i+2])
    dy = j-SEARCH+refine(*corr[j-1:j+2,i])
    return np.array([dx,dy]),peak


def main(video, crop):
    ref_crop = cv.resize(cv.imread(crop,cv.IMREAD_GRAYSCALE),(335,225),interpolation=cv.INTER_AREA)
    cap = cv.VideoCapture(video)
    frames,times,errors = [],[],[]
    while True:
        ok,f = cap.read()
        if not ok:
            break
        gray = cv.cvtColor(f,cv.COLOR_BGR2GRAY)
        diff = gray[280:390,255:435].astype(float)-ref_crop[45:155,65:245]
        errors.append(np.abs(diff).mean())
        if len(times)<110:
            frames.append(f)
        times.append(cap.get(cv.CAP_PROP_POS_MSEC)*.001)
    cap.release()
    anchor = int(np.argmin(errors))
    print("Anchor",anchor,times[anchor],"best/next MAE",sorted(errors)[:2])
    seconds = np.asarray(times)-times[anchor]+3.065
    reference = np.where(seconds>=.45)[0][0]
    reg = np.where((seconds>=.4)&(seconds<2.7))[0]
    tr = replay.Trial("W8")
    fig,axs = plt.subplots(3,1,figsize=(9,7),layout="constrained",sharex=True)
    rows=[]
    for k,xy in PATCHES.items():
        for channel in (1,2):
            series = [match(frames[i][:,:,channel],frames[reference][:,:,channel],xy) for i in reg]
            vec = np.array([s[0] for s in series])
            scores = np.array([s[1] for s in series])
            # Center on the pre-rise interval to remove constant subpixel-estimator bias.
            t = seconds[reg]
            vec -= np.median(vec[(t>=.5)&(t<1.20)],axis=0)
            rows.extend((int(i),s,k,channel,*d,q) for i,s,d,q in zip(reg,t,vec,scores))
            if channel==2:
                for axis,ax in enumerate(axs[:2]):
                    ax.plot(t,vec[:,axis],".-",ms=3,label=k)
    with (HERE/"early-cube-track.csv").open("w",newline="") as f:
        w = csv.writer(f,lineterminator="\n")
        w.writerow(["frame","approx_command_s","patch","channel_BGR_index","dx_px","dy_px","normalized_correlation"])
        w.writerows(rows)
    c = tr.cube
    t = c["sec"][::3]-tr.zero
    xyz = np.stack([c[k] for k in "xyz"],axis=1).reshape(-1,3,3)/8192
    rest = np.median(xyz[(t>=-.8)&(t<0)].reshape(-1,3),axis=0)
    mag = np.linalg.norm(xyz-rest,axis=2).max(axis=1)
    axs[2].plot(t,mag,"k.-",ms=3,label="Cube packet")
    lift = axs[2].twinx()
    lift.plot(tr.state["sec"]-tr.zero,tr.state["lift"],color="tab:blue",alpha=.65)
    lift.set_ylabel("Lift height (mm)")
    for ax,label in zip(axs,("Image x change (px)","Image y change (px)","Cube departure (g)")):
        ax.set_ylabel(label)
        ax.axvspan(1.35,1.58,color=".8",alpha=.4)
        ax.set_xlim(.4,2.7)
    axs[0].legend(ncol=4,fontsize=8)
    axs[2].set_xlabel("Seconds after wheelstand request · camera aligned by precontact frame")
    fig.savefig(HERE/"early-cube-track.png",dpi=160)
    sample = frames[reference].copy()
    for k,(x,y) in PATCHES.items():
        cv.rectangle(sample,(x-TEMPLATE,y-TEMPLATE),(x+TEMPLATE,y+TEMPLATE),(80,230,120),1)
    cv.imwrite(str(HERE/"early-cube-patches.jpg"),sample[250:400,270:470])


if __name__=="__main__":
    main(*sys.argv[1:])

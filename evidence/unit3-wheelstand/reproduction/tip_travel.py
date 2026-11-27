"""Measure screen-space fork travel independently of its stationary projection offset."""
from pathlib import Path
import csv
import sys

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

from fork_projection import PICKS
import replay

HERE=Path(__file__).resolve().parent


def track(video):
    cap=cv.VideoCapture(video)
    images={}
    times={}
    for frame in range(77):
        ok,img=cap.read()
        assert ok
        if frame>=64:
            images[frame]=cv.cvtColor(img,cv.COLOR_BGR2GRAY)
            times[frame]=cap.get(cv.CAP_PROP_POS_MSEC)*.001-5.640+3.065
    cap.release()
    points={70:np.array(PICKS[70],np.float32).reshape(2,1,2)}
    fb={70:np.zeros(2)}
    settings=dict(winSize=(15,15),maxLevel=2,criteria=(cv.TERM_CRITERIA_EPS|cv.TERM_CRITERIA_COUNT,30,.01))
    for frames in (range(69,63,-1),range(71,77)):
        prev=70
        for frame in frames:
            p,ok,_=cv.calcOpticalFlowPyrLK(images[prev],images[frame],points[prev],None,**settings)
            back,rev,_=cv.calcOpticalFlowPyrLK(images[frame],images[prev],p,None,**settings)
            assert ok.all() and rev.all()
            fb[frame]=np.linalg.norm(back-points[prev],axis=2).ravel()
            points[frame]=p
            prev=frame
    return times,{k:v.reshape(2,2) for k,v in points.items()},fb


def main(video):
    times,observed,fb=track(video)
    rows=np.genfromtxt(HERE/"fork-projection-dock-edges.csv",delimiter=",",names=True)
    tr=replay.Trial("W8")
    fig,axs=plt.subplots(1,2,sharey=True,figsize=(10,4),layout="constrained")
    with (HERE/"tip-travel.csv").open("w",newline="") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["frame","time_s","side","encoder_mm","tracked_x","tracked_y","step_round_trip_error_px","tracked_minus_hand_px","static_corrected_x_error_px","static_corrected_y_error_px"])
        for side,ax in enumerate(axs):
            sub=rows[rows["side"]==side]
            static=sub[sub["frame"]==70][0]
            predicted_ref=np.array([static["pred_x_px"],static["pred_y_px"]])
            anchor=observed[70][side]
            frames=sorted(observed)
            t=np.array([times[k] for k in frames])
            obs=np.array([observed[k][side] for k in frames])
            ax.plot(t,obs[:,1]-anchor[1],"k.-",label="Video tracking")
            sim=[]
            for frame in frames:
                xy=observed[frame][side]
                manual=np.nan
                err=np.full(2,np.nan)
                if frame in PICKS:
                    hand=np.array(PICKS[frame][side])
                    manual=np.linalg.norm(xy-hand)
                    ax.plot(times[frame],hand[1]-anchor[1],"+",color="tab:green",label="Hand pick" if frame==64 else None)
                    row=sub[sub["frame"]==frame][0]
                    delta=np.array([row["pred_x_px"],row["pred_y_px"]])-predicted_ref
                    sim.append([times[frame],delta[1]])
                    err=delta-(xy-anchor)
                w.writerow([frame,times[frame],side,tr.obs(times[frame],"lift_mm"),*xy,fb[frame][side],manual,*err])
            sim=np.array(sim)
            ax.plot(sim[:,0],sim[:,1],"o--",color="tab:orange",label="Encoder projection")
            ax.set_title(("Near" if side==0 else "Far")+" upright")
            ax.set_xlabel("Seconds after command")
            ax.legend()
    axs[0].set_ylabel("Vertical travel relative to frame 70 (movie px)")
    fig.savefig(HERE/"tip-travel.png",dpi=160)


if __name__=="__main__":
    main(sys.argv[1])

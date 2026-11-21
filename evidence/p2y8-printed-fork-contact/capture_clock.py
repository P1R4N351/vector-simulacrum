"""Test robot capture intervals against host receipt times, then rescore stripe position."""
import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Slerp, Rotation
from front_edge import here, states, rows, reconstruct, project_pixels
from recovered_marker import recovered
from optical_clearance import quads
from stripe_position import outer_edge, windows
from joint_marker import residual_segment

edges = np.genfromtxt(here.parent/"p2y8-cube-camera-scale/edges.csv",names=True,delimiter=",")
scale = 10379.5644
# Robot timestamps from the source JPEG filenames for these three trial 1 frames.
stamps = [2015079,2017669,2018068]


def approach(trial,clock,lag):
    q=edges[(edges["trial"]==trial)&(edges["seconds"]<2.75)]
    t=q["seconds"]
    capture=q["capture_ms"]*.001
    offset=np.median(t-capture)
    source=t if clock=="receipt" else capture+offset
    width=np.hypot(q["u1"]-q["u0"],q["v1"]-q["v0"])
    optic=scale*(1/np.median(width[t<.5])-1/width)
    s=states[states["trial"]==trial]
    yaw=np.deg2rad(s["yaw_deg"][0])
    distance=(s["x_mm"]-s["x_mm"][0])*np.cos(yaw)+(s["y_mm"]-s["y_mm"][0])*np.sin(yaw)
    pred=np.interp(source+1.5-lag,s["sec"],distance)
    return (optic-pred)[t>1],offset


def main():
    timing={}
    with (here/"front-edge/clock-travel.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["clock","lag_s","trial","frames","approach_rmse_mm"])
        for clock in ("receipt","capture"):
            def loss(lag):
                return np.concatenate([approach(i,clock,lag[0])[0] for i in (1,2)])
            lag=least_squares(loss,[.15],loss="soft_l1",bounds=(-.45,.45)).x[0]
            timing[clock]=(lag,approach(1,clock,lag)[1])
            for trial in range(1,6):
                err,_=approach(trial,clock,lag)
                w.writerow([clock,lag,trial,len(err),np.sqrt(np.mean(err**2))])
    times=rows["sec"]
    frames=recovered()
    extra_times, extra_corners=zip(*frames[::2])
    biases=[]
    for t,q in frames[1::2]:
        control=quads[np.argmin(abs(times-t))]
        biases.append(control-q)
        print(f"{t:.3f} s control corner errors (px):",np.linalg.norm(control-q,axis=1))
    all_times=np.r_[times,extra_times]
    order=np.argsort(all_times)
    datasets={"detected":(times,quads),
              "recovered":(all_times[order],np.concatenate([quads,extra_corners])[order])}
    # First control learns the difference between line intersections and detector
    # corners. The second control measures transfer without informing the fit.
    print("held-out corrected corner errors (px):", np.linalg.norm(biases[1]-biases[0],axis=1))
    datasets["control-adjusted"]=(all_times[order],np.concatenate([quads,extra_corners+biases[0]])[order])
    solutions={}
    for key,(ts,qs) in datasets.items():
        poses=reconstruct(-1.499,473.11,True,qs,ts)
        spin=Slerp(ts,Rotation.from_quat([r.as_quat() for r,_,_ in poses]))
        solutions[key]=(ts,spin,np.array([d for _,d,_ in poses]))
    fig,ax=plt.subplots(figsize=(8,4))
    with (here/"front-edge/clock-position.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["view","clock","sample","source_sec","pose_sec","bracket_gap_sec","stripe_median_error_px"])
        for clock,(lag,offset) in timing.items():
            for sample in ("nearest","interpolated","recovered","control-adjusted"):
                times,rotate,shifts=solutions[sample if sample in datasets else "detected"]
                errs=[]
                for (name,receipt,*box),stamp in zip(windows,stamps):
                    t=(receipt if clock=="receipt" else stamp*.001+offset)-lag
                    j=np.searchsorted(times,t)
                    s=times[np.argmin(abs(times-t))] if sample=="nearest" else t
                    delta=[np.interp(s,times,col) for col in shifts.T]
                    seg=project_pixels(t,rotate(s),delta)
                    img=cv2.imread(str(here/f"front-edge/{name}.png"))
                    err=np.median(np.linalg.norm(residual_segment(outer_edge(img,box),seg),axis=1))
                    w.writerow([name,clock,sample,t,s,times[j]-times[j-1],err])
                    errs.append(err)
                ax.plot(["before","raised","lowering"],errs,"o-",label=f"{clock}, {sample}")
    ax.set(ylabel="Median stripe position error (px)",title="Onboard stripe position from overhead cube poses")
    ax.legend()
    fig.tight_layout()
    fig.savefig(here/"front-edge/clock.png",dpi=150)


if __name__=="__main__":
    main()

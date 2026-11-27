"""Compare position at the hardware edge and time of the simulated edge separately."""
from collections import defaultdict
import csv
import gzip
import numpy as np
import matplotlib.pyplot as plt

def rows(path):
    with open(path) as src:
        return list(csv.DictReader(src))

def optical_data():
    optics={r["session"]:(float(r["reference_s"]),float(r["last_on_s"]),float(r["interpolated_midpoint_mm"])) for r in rows("../unit3-cube-range/release-optics.csv")}
    frames=defaultdict(list)
    for r in rows("../unit3-cube-range/release-frames.csv"):
        frames[r["session"]].append((float(r["frame_s"]),float(r["forward_mm"])))
    h=np.genfromtxt("../unit3-w34-return/departure-hubs.csv",names=True,delimiter=",")
    distance=np.mean([(h[k][0]-h[k]+h["static_x"]-h["static_x"][0])*48/124 for k in ["front_x","rear_x"]],axis=0)
    with gzip.open("../unit3-w34-return/continuation.csv.gz","rt") as f:
        states=list(csv.DictReader(f))
    i=next(i for i in range(1,len(states)) if int(states[i-1]["status"])&4096 and not int(states[i]["status"])&4096)
    last_on=float(states[i-1]["receipt_s"])-float(states[i]["receipt_s"])
    optics["W35"]=(h["t_s"][0],last_on,np.interp(last_on/2,h["t_s"],distance))
    frames["W35"]=list(zip(h["t_s"],distance))

    return optics,frames

def main():
    optics,frames=optical_data()
    groups=defaultdict(list)
    for r in rows("traces.csv"):
        groups[r["trial"],r["mode"],int(r["offset"])].append([float(r[k]) for k in ["t","x_mm","on"]])
    fig,panels=plt.subplots(2,4,layout="constrained",figsize=(13,6))
    panels.flat[-1].axis("off")
    with open("scores.csv","w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["trial","mode","offset_mm","mid_camera_mm","mid_sim_mm","mid_error_mm","sim_off_s","edge_gap_ms","release_travel_mm","camera_frame_rmse_mm"])
        for ax,(name,(ref,on,physical)) in zip(panels.flat,optics.items()):
            times,positions=np.asarray(frames[name]).T
            keep=(times>=ref)&(times<=.4)
            ax.plot(times[keep],positions[keep],"k.-",label="Camera")
            ax.axvspan(on,0,color="gray",alpha=.2)
            for mode in ["controller","shaft","linear"]:
                for offset in [0,6]:
                    t,x,flag=np.asarray(groups[name,mode,offset]).T
                    x-=np.interp(ref,t,x)
                    j=np.flatnonzero(flag==0)[0]
                    mid=np.interp(on/2,t,x)
                    rmse=np.mean((np.interp(times[keep],t,x)-positions[keep])**2)**.5
                    w.writerow([name,mode,offset,physical,mid,mid-physical,t[j],1000*max(on-t[j],t[j],0),x[j],rmse])
                    if offset==6:
                        ax.plot(t,x,label=mode)
                        ax.scatter(t[j],x[j],s=15,marker="x")
            ax.set(xlim=(max(-.65,ref),.4),ylim=(-1,22),title=name,xlabel="Seconds from first off",ylabel="Forward travel (mm)")
        panels.flat[0].legend(fontsize=8)
    fig.savefig("departures.png",dpi=155)

if __name__ == "__main__":
    main()

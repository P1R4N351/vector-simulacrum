"""Account for dock-axis travel up to the measured 3° nose-down crossing."""
import csv

import matplotlib.pyplot as plt
import numpy as np

from mount_shaft_replay import replay
from recorded_mount import ROOT, read_trial, table


def inward(t, wheels, heading):
    rate = -wheels * np.cos(np.deg2rad(heading))
    return np.r_[0, np.cumsum(.5 * (rate[:-1] + rate[1:]) * np.diff(t))]


def main():
    cases = [read_trial(c, o) for c, o in zip(table("return-scores.csv"),table("mount-target-inputs.csv"),strict=True)]
    pitch = table("mount-pitch.csv")
    scores = []
    fig, axes = plt.subplots(1,5,figsize=(14,3.8),sharey=True,layout="constrained")
    for mode in ["Held command","Linear command","Immediate linear","Shaft 0.8"]:
        data = replay([trial for trial, _ in cases],mode)
        for ax, (trial, real) in zip(axes,cases,strict=True):
            name = trial["name"]
            end = pitch[(pitch["session"]==name)&(pitch["model"]=="canonical")]["gyro_cross_s"].item()
            src,out,_,yaw,wheels = real
            take = src<=end
            t=src[take]
            sdk=out[0]-out[take]
            wheel=inward(t,wheels[take].mean(axis=1),yaw[take])
            run=data[data["session"]==name]
            st=np.r_[run["t_s"][run["t_s"]<end], end]
            def sample(field):
                return np.interp(st,run["t_s"],run[field])
            shaft=inward(st,(sample("shaft_l")+sample("shaft_r"))/2,sample("yaw_deg"))
            chassis=run["out_mm"][0]-sample("out_mm")
            assert np.isclose(t[-1],end)
            scores.append([name,mode,t[0],end,sdk[-1],wheel[-1],shaft[-1],chassis[-1],
                           sdk[-1]-wheel[-1],wheel[-1]-shaft[-1],shaft[-1]-chassis[-1],next(iter(run["t_s"][run["on"]==1]),"")])
            if mode=="Held command":
                ax.plot(t-end,sdk,"k.-",label="SDK position")
                ax.plot(t-end,wheel,"k:",label="Recorded wheels")
                ax.plot(st-end,shaft,"--",label="Simulated shafts (held)")
            ax.plot(st-end,chassis,label=mode)
            ax.set_title(f"Return {name}")
            ax.set_xlabel("Time to pitch crossing (s)")
    axes[0].set_ylabel("Inward travel (mm)")
    axes[0].legend(fontsize=7)
    fig.savefig(ROOT/"mount-approach-travel.png",dpi=160)
    with (ROOT/"mount-approach-travel.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow("session mode start_s pitch_cross_s sdk_mm wheels_mm shaft_mm chassis_mm sdk_minus_wheels_mm wheels_minus_shaft_mm shaft_minus_chassis_mm first_contact_s".split())
        writer.writerows(scores)


if __name__=="__main__":
    main()

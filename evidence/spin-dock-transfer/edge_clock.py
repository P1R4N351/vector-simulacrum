"""Reconstruct interval-reported shaft speeds and bound the final detected edge."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

states = np.genfromtxt(sys.argv[1],delimiter=",",names=True,encoding="utf8",dtype=None)
pitch = .96 * 29 * .25 * np.pi / 172.3
tick_ms = 1000 / (48e6 * 256)
fig, axes = plt.subplots(2,2,figsize=(10,6),sharex=True,layout="constrained")
with open("edge-clock.csv","w",newline="") as stream:
    out = csv.writer(stream,lineterminator="\n")
    out.writerow(["turn","side","samples","report_mae_mmps","frame_count_mae_mmps",
                  "max_report_error_mmps","nominal_low_ms","nominal_high_ms","rounded_low_ms","rounded_high_ms"])
    for turn,ax in zip(("ccw_1","cw_1","ccw_2","cw_2"),axes.flat):
        s = states[states["turn"]==turn]
        t = s["ms_from_last_edge"]
        for side,sign,color in (("left",1,"#1262af"),("right",-1,"#bd5032")):
            pos = s[side+"_count"]
            fresh = np.r_[False,np.diff(pos)!=0]
            reported=s[side+"_mmps"]
            interval_ms = s[side+"_ticks"]*tick_ms
            n=s[side+"_delta"]
            speed=np.zeros(len(t))
            idle=5
            for j in range(1,len(t)):
                idle=0 if fresh[j] else idle+1
                if interval_ms[j] and idle<5:
                    speed[j]=np.trunc(sign*n[j]*pitch*1000/interval_ms[j])
            frame_speed=np.r_[0,sign*np.diff(pos)*pitch*1000/np.diff(t)]
            use=(t>=-100)&(t<=100)
            # The interval on each fresh frame is time from previous final edge
            # to this final edge. t bounds acquisition to its update frame,
            # under a common phase between the frame and state clock.
            idx=np.flatnonzero((t>=-100)&(t<=0)&fresh)
            assert np.all(np.diff(s["frame"][use])==1)
            assert all(pos[k]-pos[k-1]==n[k] for k in idx)
            rel=np.cumsum(interval_ms[idx]); rel-=rel[-1]
            low=max(t[idx-1]-rel); high=min(t[idx]-rel)
            out.writerow([turn,side,sum(use),np.mean(abs(speed[use]-reported[use])),
                          np.mean(abs(frame_speed[use]-reported[use])),
                          max(abs(speed[use]-reported[use])),low,high,low-.5,high+.5])
            ax.plot(t[use],abs(reported[use]),color=color,alpha=.5,lw=3,label=side+" reported")
            ax.plot(t[use],abs(speed[use]),color=color,ls=":",lw=1.4,label=side+" from intervals")
            if low-.5<=high+.5:
                ax.axvspan(low-.5,high+.5,color=color,alpha=.20)
        ax.axvline(0,color=".4",lw=.8)
        ax.set(title=turn.upper().replace("_"," "),xlabel="Time from final changing frame (ms)",ylabel="Shaft speed magnitude (mm/s)")
axes.flat[0].legend(fontsize=7)
fig.savefig("edge-clock.png",dpi=180)

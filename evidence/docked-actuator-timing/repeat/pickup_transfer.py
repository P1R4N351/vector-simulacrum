"""Compare the same absolute wrist travel in direct raises and cube pickups."""
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
evidence = here.parents[1]


def direct(folder):
    rows = list(map(json.loads,(folder/"unit3.jsonl").read_text().splitlines()))
    with (folder/"moves.csv").open() as stream:
        for row in csv.DictReader(stream):
            if (row["bot"],row["source"],row["joint"],row["direction"]) != ("unit 3","unit 3","lift","up"):
                continue
            start = float(row["command_s"])
            states = [r for r in rows if r["k"]=="s" and 0<=r["t"]-start<1.5]
            yield np.array([(r["t"]-start,r["lift"]) for r in states]).T


def crossing(t,h,target):
    idx = np.flatnonzero((h[:-1]<target)&(h[1:]>=target))[0]
    return np.interp(target,h[idx:idx+2],t[idx:idx+2]),t[idx],t[idx+1]


def main():
    strokes = [(name,*trace) for name,folder in [("Docked direct",here),("Off-dock direct",here.parent)]
               for trace in direct(folder)]
    for name,path,clock,height in [
        ("Earlier pickup",evidence/"unit3-cube-accelerometer/pickup-path.csv","time_s","lift_mm"),
        ("R5 pickup",evidence/"unit3-wheelstand/R5/approach.csv","sec","lift")]:
        a = np.genfromtxt(path,names=True,delimiter=",")
        use = (a[clock]>=1.8)&(a[clock]<5.5)
        strokes.append((name,a[clock][use],a[height][use]))
    fig,ax = plt.subplots(figsize=(8,4.5),layout="constrained")
    colors = {n:f"C{i}" for i,n in enumerate(dict.fromkeys(s[0] for s in strokes))}
    seen = set()
    with (here/"pickup-transfer.csv").open("w") as f:
        w = csv.writer(f,lineterminator="\n")
        w.writerow(["condition","stroke","40_to_60_ms","60_to_80_ms","40_to_80_ms",
                    "40_to_80_lower_bracket_ms","40_to_80_upper_bracket_ms"])
        for i,(name,t,h) in enumerate(strokes,1):
            first,mid,last = [crossing(t,h,target) for target in (40,60,80)]
            # Wide interval covers any crossing between consecutive SDK receipts.
            durations = [mid[0]-first[0],last[0]-mid[0],last[0]-first[0],
                         last[1]-first[2],last[2]-first[1]]
            w.writerow([name,i,*np.round(1000*np.array(durations),1)])
            ax.plot(t-first[0],h,".-",lw=1,ms=4,color=colors[name],label=None if name in seen else name)
            seen.add(name)
    ax.set(xlim=(-.12,.8),ylim=(28,96),xlabel="Seconds from interpolated 40 mm crossing",
           ylabel="Recorded wrist height (mm)")
    ax.axhline(80,ls=":",color=".45")
    ax.legend()
    fig.savefig(here/"pickup-transfer.png",dpi=160)


if __name__=="__main__":
    main()

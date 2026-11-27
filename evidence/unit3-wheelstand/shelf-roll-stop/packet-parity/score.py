"""Compare emitted samples at their original packet times, then score the physical impact slots."""
import csv
import gzip
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(sys.argv[1])

def load_packets(run, mode):
    name = f"{run}-{mode}.csv"
    plain = root / name
    if plain.is_file():
        return np.loadtxt(plain, delimiter=",", skiprows=1, dtype=int)
    return np.loadtxt(root / (name + ".gz"), delimiter=",", skiprows=1, dtype=int)

here = Path(__file__).resolve().parent
captures = [
    ("shelf", "../S22B/cube.csv.gz", 37.020336, 1),
    ("pickup", "../../R5/cube.csv.gz", 39.980848, 0),
]
files = ("software", "firmware")
rows = []
fig, axes = plt.subplots(2, 2, figsize=(10, 6), sharex=True, layout="constrained")
for axrow, (run, source, timestamp, slot) in zip(axes, captures):
    with gzip.open(here / source, "rt") as stream:
        records = list(csv.DictReader(stream))
    first = next(i for i, r in enumerate(records)
                 if float(r["sec"]) == timestamp and int(r["slot"]) == slot)
    obs = np.array([[int(r[k]) for k in "xyz"] for r in records[first:first+5]]) / 8192
    pair = [load_packets(run, mode)
            for mode in files]
    # Distinguish a packet schedule difference from different content at a shared time.
    keyed = [{tuple(r[[0,1,2,3,4,6]]): r[[5,7,8,9]] for r in arr} for arr in pair]
    assert all(len(k) == len(a) for k,a in zip(keyed,pair))
    common = keyed[0].keys() & keyed[1].keys()
    print(run, "slot rows in each, common, identical XYZ, identical taps",
          *map(len,keyed),len(common),
          sum(np.array_equal(keyed[0][k][1:],keyed[1][k][1:]) for k in common),
          sum(keyed[0][k][0]==keyed[1][k][0] for k in common))
    for ax, mode, arr in zip(axrow,files,pair):
        matched = []
        for v,h,phase in np.unique(arr[:,:3],axis=0):
            trace = arr[np.all(arr[:,:3]==[v,h,phase],axis=1)]
            assert np.all(trace[:,6] == np.arange(len(trace))%3)
            xyz = trace[:,7:] / 8192
            for i in range(slot,len(trace)-4,3):
                low = np.linalg.norm(xyz[i:i+2],axis=1).max()
                if low >= .05:
                    continue
                err = np.sqrt(np.mean((xyz[i+2:i+5]-obs[2:])**2))
                matched.append((err,v,h,phase,xyz[i:i+5]))
                rows.append((run,mode,v,h,phase,trace[i,4],err))
        best=min(matched,key=lambda item:item[0])
        if run == "shelf" and mode == "software":
            frozen = best[1:4]
        if run == "pickup":
            print("Held-out pickup windows at shelf-selected setting",
                  sum(tuple(item[1:4]) == frozen for item in matched))

        print(run,mode,len(matched),"best",best[:4])
        for j,color in enumerate(("tab:blue","tab:orange","tab:green")):
            ax.plot(range(5),obs[:,j],color=color,marker="o",label="XYZ"[j])
            ax.plot(range(5),best[-1][:,j],color=color,linestyle="--",marker=".")
        ax.set_title(f"{run.capitalize()} · {mode} · {best[0]:.2f} g RMS")
        ax.set_xticks(range(5))
axes[0,0].legend(ncol=3)
for ax in axes[:,0]:
    ax.set_ylabel("Acceleration (g)")
for ax in axes[-1]:
    ax.set_xlabel("Successive recorded sample slots")
fig.suptitle("Recorded (solid) and best-case simulated impact (dashed)")
fig.savefig(here/"impacts.png",dpi=160)
with (here/"matches.csv").open("w") as stream:
    w=csv.writer(stream,lineterminator="\n")
    w.writerow(("run","mode","velocity_mmps","gap_mm","phase_ms","first_packet_ms","xyz_rms_g"))
    w.writerows(rows)

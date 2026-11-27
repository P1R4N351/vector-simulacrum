"""Compare stock edge periods against the best-case 5 ms packet clock on native stops."""
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

groups = defaultdict(list)
with open(Path("../hw-rec-20261119/unit2-native-stops.csv")) as f:
    for row in csv.DictReader(f):
        groups[row["turn"]].append(row)
scale = 0.96 * 29 * np.pi / (4 * 172.3)
clock = 48000000 * 256
fig, panels = plt.subplots(2, 2, figsize=(10, 6), layout="constrained", sharex=True)
with open("encoder-clock.csv", "w") as out:
    writer = csv.writer(out, lineterminator="\n")
    writer.writerow(["turn","side","edge_frames","off_5ms_grid","native_word_mae_mmps","frame_grid_mae_mmps","period_retained_after_delta_zero"])
    for turn, rows in groups.items():
        for k,side in enumerate(("left","right")):
            t,pos,delta,ticks,speed = np.array([[int(r[key]) for r in rows] for key in
                ("ms_from_last_edge",side+"_count",side+"_delta",side+"_ticks",side+"_mmps")])
            seq = np.array([int(r["frame"]) for r in rows])
            fresh = (np.diff(pos,prepend=pos[0])!=0) & (np.diff(seq,prepend=seq[0])==1) & (ticks>0)
            factor = scale * (-1 if k else 1)
            exact = np.trunc(factor * delta[fresh] * clock / ticks[fresh])
            quantized = np.maximum(1,np.round(ticks[fresh]/(.005*clock))) * .005
            grid = np.trunc(factor * delta[fresh]/quantized)
            held = np.count_nonzero((delta==0)&(ticks!=0)&(t>0))
            writer.writerow([turn,side,sum(fresh),sum(ticks[fresh]%(clock//200)!=0),
                             np.mean(abs(exact-speed[fresh])),np.mean(abs(grid-speed[fresh])),held])
            if turn.endswith("_2"):
                ax = panels[k, 0 if turn.startswith("ccw") else 1]
                ax.plot(t[fresh],speed[fresh],"k.",ms=4,label="Native reported")
                ax.plot(t[fresh],grid,"x",ms=3,label="Rounded period")
                ax.plot(t[fresh],exact,"+",ms=3,label="Native edge period")
                ax.set(title=turn.replace("_2","").upper()+" "+side, ylabel="Wheel speed (mm/s)",
                       xlabel="Milliseconds from last edge")
panels[0,0].legend(fontsize=8)
fig.savefig("encoder-clock.png",dpi=150)

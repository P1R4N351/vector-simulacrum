"""Compare two decompositions of the same fork against the R5 optical raise."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

from pickup_onset import action_time, read

here = Path(__file__).resolve().parent
glyph = read(here/"glyph-raise.csv")
lights = read(here/"raise-lights.csv")
start = action_time(here/"actions.csv","pickup for inward transport","sec")
times = np.interp(glyph["frame"],lights["frame"],lights["receipt_s"])-start
fig, axes = plt.subplots(2,1,figsize=(8,6),layout="constrained",sharex=True)
rows = []
for size, path in zip((58,173,173),sys.argv[1:],strict=True):
    folder = Path(path)
    trace = read(folder/"pickup-replay.csv")
    loads = np.loadtxt(folder/"contact-load.csv",delimiter=",",skiprows=1)
    total = loads[:,3:6].sum(axis=1)
    rising = (loads[:,0]>=2.559)&(loads[:,1]<90)&(loads[:,0]<3.3)
    hold = (trace["sec"]>=5.5)&(trace["sec"]<=7.9)
    angles = np.interp(times,trace["sec"],trace["tilt_deg"])
    step_ms = 1000*np.median(np.diff(loads[:,0]))
    rows.append([size,round(step_ms,3),*angles,total[rising].mean(),loads[rising,7].max(),
                 np.mean(trace["floor_contact"][hold]),np.median(trace["tilt_deg"][hold])])
    axes[0].plot(trace["sec"],trace["tilt_deg"],label=f"{size} pieces, {step_ms:.3g} ms")
    bin_size = round(.01/np.median(np.diff(loads[:,0])))
    count = len(loads)//bin_size*bin_size
    axes[1].plot(loads[:count,0].reshape(-1,bin_size).mean(axis=1),
                 total[:count].reshape(-1,bin_size).mean(axis=1))
axes[0].scatter(times,glyph["normal_change_deg"],color="black",label="Physical glyph, shelf reference")
axes[0].set_ylabel("Cube tilt (°)")
axes[0].legend(fontsize=8)
axes[1].set(xlabel="Seconds after pickup command",ylabel="Upward fork and rail force / cube weight",
            yscale="symlog",xlim=(2.2,3.55))
fig.savefig(here/"fork-resolution.png",dpi=160)
with (here/"fork-resolution.csv").open("w") as stream:
    writer=csv.writer(stream)
    writer.writerow(["pieces","step_ms",*[f"frame_{i}_deg" for i in glyph["frame"]],
                     "mean_raise_force_weight","max_raise_penetration_mm","hold_floor_fraction","hold_deg"])
    writer.writerows(rows)

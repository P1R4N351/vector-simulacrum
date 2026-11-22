"""Compare floor-only sliding-friction controls with the early R5 optical tilt."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

from pickup_onset import action_time, read

here = Path(__file__).resolve().parent
lights = read(here/"raise-lights.csv")
glyph = read(here/"glyph-raise.csv")
start = action_time(here/"actions.csv","pickup for inward transport","sec")
ticks = np.interp(glyph["frame"],lights["frame"],lights["receipt_s"])-start
fig,axes = plt.subplots(2,1,figsize=(8,6),layout="constrained",sharex=True)
rows = []
for label,folder in zip(["Original","1× split control","0.25×","0.5×","2×"],sys.argv[1:],strict=True):
    sim = read(Path(folder)/"pickup-replay.csv")
    if label == "Original":
        original = sim
    dt = np.max(abs(sim["tilt_deg"]-original["tilt_deg"]))
    positions = [sim[k] for k in ("dx_mm","dy_mm")]
    delta = [p-np.interp(ticks[0],sim["sec"],p) for p in positions]
    travel = np.hypot(*delta)
    angles = np.interp(ticks,sim["sec"],sim["tilt_deg"])
    hold = (sim["sec"]>=5.5)&(sim["sec"]<=7.9)
    rows.append([label,dt,angles[0],angles[2],angles[3],np.interp(ticks[3],sim["sec"],travel),
                 np.median(sim["tilt_deg"][hold]),np.mean(sim["floor_contact"][hold])])
    axes[0].plot(sim["sec"],sim["tilt_deg"],label=label)
    axes[1].plot(sim["sec"],travel)
axes[0].scatter(ticks,glyph["normal_change_deg"],color="black",label="Physical glyph (from frame 85)")
axes[0].legend(fontsize=8,ncol=2)
axes[0].set_ylabel("Cube tilt (°)")
axes[1].set(xlabel="Seconds after pickup command",ylabel="Simulated center travel in shelf plane\nfrom frame 85 (mm)",xlim=(ticks[0]-.15,ticks[-1]+.1))
fig.savefig(here/"shelf-friction.png",dpi=160)
with (here/"shelf-friction.csv").open("w") as stream:
    w=csv.writer(stream)
    w.writerow("condition max_trace_delta_deg frame85_deg frame91_deg frame94_deg frame85_to_94_xy_mm hold_deg hold_floor_fraction".split())
    w.writerows(rows)

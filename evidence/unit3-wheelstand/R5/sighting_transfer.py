"""Score each R5 sighting restart against the independently picked top-face poses."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

from pickup_onset import action_time, read

here = Path(__file__).resolve().parent
start = action_time(here/"actions.csv", "pickup for inward transport", "sec")
glyph = read(here/"glyph-raise.csv")
lights = read(here/"raise-lights.csv")
times = np.interp(glyph["frame"], lights["frame"], lights["receipt_s"]) - start
observed = glyph["normal_change_deg"]
sightings = read(here/"sightings.csv")
fig, ax = plt.subplots(figsize=(8, 4), layout="constrained")
rows = []
for arg in sys.argv[1:]:
    number, folder = arg.split("=",1)
    sighting = sightings[int(number)]
    trace = read(Path(folder)/"pickup-replay.csv")
    model = np.interp(times,trace["sec"],trace["tilt_deg"])
    hold = (trace["sec"]>=5.5)&(trace["sec"]<=7.9)
    ax.plot(trace["sec"],trace["tilt_deg"],label=f'Sighting {sighting["sec"]:+.2f} s')
    rows.append([number,sighting["sec"],sighting["x"],sighting["y"],*model,
                 np.median(trace["tilt_deg"][hold]),np.mean(trace["floor_contact"][hold])])
ax.scatter(times, observed, color="black", zorder=5, label="Physical glyph")
ax.set(xlim=(2.2,3.65),xlabel="Seconds from pickup command",ylabel="Cube tilt from shelf (°)")
ax.legend(fontsize=8)
fig.savefig(here/"sighting-transfer.png",dpi=160)
with (here/"sighting-transfer.csv").open("w") as stream:
    w=csv.writer(stream)
    w.writerow(["sighting","receipt_s","x_mm","y_mm",*[f"frame_{i}_deg" for i in glyph["frame"]],"hold_deg","hold_floor_fraction"])
    w.writerows(rows)

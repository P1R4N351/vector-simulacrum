"""Score travel at contact and distance outside the hardware sampling interval."""
import csv
import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

def read(path):
    with open(path) as stream:
        return list(csv.DictReader(stream))

optics = read(sys.argv[2])
traces = read(sys.argv[1])
fig, (timing, travel) = plt.subplots(2, 1, figsize=(8, 6), layout="constrained")
with open("transfer.csv", "w") as src:
    writer = csv.writer(src, lineterminator="\n")
    writer.writerow(["session", "shift_mm", "last_on_ms", "off_ms", "signed_bracket_miss_ms", "travel_error_mm"])
    for shift, label in [(0, "Nominal"), (6, "A–C fit"), (7.3984375, "W35 camera")]:
        times, errors = [], []
        for row in optics:
            session = row["session"]
            group = [r for r in traces if r["trial"] == session and float(r["offset"]) == shift]
            t, x, flag = np.array([[float(r[k]) for k in ("t","x_mm","on")] for r in group]).T
            off = t[np.flatnonzero(flag == 0)[0]]
            last_on = float(row["last_on_s"])
            miss = 1000*(off - np.clip(off, last_on, 0))
            distance = np.interp(last_on/2,t,x) - np.interp(float(row["reference_s"]),t,x)
            error = distance - float(row["interpolated_midpoint_mm"])
            writer.writerow([session, shift, 1000*last_on, 1000*off, miss, error])
            times.append(miss)
            errors.append(error)
        for ax, values in [(timing,times), (travel,errors)]:
            ax.plot([r["session"] for r in optics], values, "o-",label=label)
            ax.axhline(0,color="gray",linewidth=.5)
timing.set(ylabel="Release bracket miss (ms)", title="Earlier (−) or later (+) than physical charger transition")
travel.set(ylabel="Sim minus camera travel (mm)", xlabel="Physical departure",title="Forward travel at midpoint of charger bracket")
timing.legend()
fig.savefig("transfer.png",dpi=160)

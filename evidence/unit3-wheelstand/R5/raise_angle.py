"""Align two pickups by encoder motion and compare apparent tilt by lift height."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from pickup_onset import action_time, read

here = Path(__file__).resolve().parent
earlier = here.parents[1] / "unit3-cube-accelerometer"
runs = [
    ("Earlier", read(earlier / "pickup.csv"), action_time(earlier / "action-times.csv", "pickup", "pod_unix_s"),
     read(earlier / "pickup-path.csv"), read(earlier / "free-pickup.csv")),
    ("R5", read(here / "cube.csv.gz"), action_time(here / "actions.csv", "pickup for inward transport", "sec"),
     read(here / "approach.csv"), read(here / "pickup-replay.csv")),
]
fig, axes = plt.subplots(2, 2, figsize=(10, 6), layout="constrained", sharey=True)
rows = []
for col, (label, cube, start, path, sim) in enumerate(runs):
    if label == "Earlier":
        times = cube["pod_rx_unix_ns"][::3] / 1e9 - start
        keys = ("x_counts", "y_counts", "z_counts")
        pt, ph = path["time_s"], path["lift_mm"]
        sim = sim[sim["initialization"] == "Before command"]
        st = sim["time_s"]
    else:
        times = cube["sec"][::3] - start
        keys = ("x", "y", "z")
        pt, ph = path["sec"], path["lift"]
        st = sim["sec"]
    packets = np.column_stack([cube[k] for k in keys]).reshape(-1, 3, 3)
    assert np.all(cube["slot"].reshape(-1, 3) == [0, 1, 2])
    keep = np.r_[True, np.any(np.diff(packets, axis=0) != 0, axis=(1, 2))]
    times, vectors = times[keep], packets[keep].mean(axis=1)
    reference = np.median(vectors[(times >= -1) & (times < 0)], axis=0)
    apparent = np.degrees(np.arctan2(np.linalg.norm(np.cross(vectors, reference), axis=1), vectors @ reference))
    initial = np.median(ph[(pt >= 0) & (pt < 1)])
    i = np.flatnonzero((pt > 1) & (ph > initial + 1))[0]
    onset = np.interp(initial + 1, ph[i-1:i+1], pt[i-1:i+1])
    end = pt[np.flatnonzero((pt >= onset) & (ph >= 90))[0]]
    height = np.interp(times, pt, ph)
    showing = (times >= onset - .3) & (times <= end + .4)
    model = (st >= onset - .3) & (st <= end + .4)
    axes[0,col].scatter(times[showing]-onset, apparent[showing], s=9, label="Cube accelerometer")
    axes[0,col].plot(st[model]-onset, sim["tilt_deg"][model], color="tab:orange", label="Simulated geometric tilt")
    axes[0,col].axvspan(pt[i-1]-onset, pt[i]-onset, alpha=.15, color=".4")
    axes[0,col].set(title=label, xlabel="Seconds from 1 mm lift rise")
    rising = (times >= onset) & (times < end)
    model = (st >= onset) & (st < end)
    axes[1,col].scatter(height[rising], apparent[rising], s=12)
    axes[1,col].plot(np.interp(st[model],pt,ph), sim["tilt_deg"][model], color="tab:orange")
    axes[1,col].set_xlabel("Interpolated encoder height during raise (mm)")
    for low, high in [(32,45),(45,60),(60,75),(75,90)]:
        use = rising & (height >= low) & (height < high)
        predicted = np.interp(times[use], st, sim["tilt_deg"])
        median = lambda a: np.median(a) if len(a) else ""
        rows.append([label, low, high, onset, pt[i]-pt[i-1], use.sum(), median(apparent[use]),
                     median(predicted), median(abs(apparent[use]-predicted)), median(np.linalg.norm(vectors[use],axis=1)/8192)])
axes[0,0].legend(fontsize=8)
fig.supylabel("Angle from initial shelf orientation (°)")
fig.savefig(here/"raise-angle.png", dpi=160)
with (here/"raise-angle.csv").open("w") as stream:
    writer = csv.writer(stream)
    writer.writerow("run low_mm high_mm first_rise_s encoder_bracket_s packets apparent_deg sim_deg median_abs_difference_deg force_norm_g".split())
    writer.writerows(rows)

"""Held-out SDK XY around electrical release, relative to the first rolling report in the final contact interval."""
import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
fig, axes = plt.subplots(2, 3, figsize=(10, 6), layout="constrained", sharex=True)
rows = []
for name, ax in zip("ABCDEF", axes.flat):
    with gzip.open(here.parent/"unit3-physical-repeats"/name/"robot.csv.gz", "rt") as stream:
        data = list(csv.DictReader(stream))
    docked = np.array([bool(int(r["status"]) & 4096) for r in data])
    t = np.array([float(r["receipt_s"]) for r in data])
    wheels = np.array([[float(r[k]) for k in ("left_mmps", "right_mmps")] for r in data])
    xy = np.array([[float(r[k]) for k in ("x_mm","y_mm")] for r in data])
    origin = np.array([int(r["origin"]) for r in data])
    release = next(i for i in range(1,len(t)) if docked[i-1] and not docked[i] and
                   np.any(wheels[i-1]))
    start = release
    while start > 0 and docked[start-1] and np.any(wheels[start-1]):
        start -= 1
    assert release - start >= 2
    end = np.searchsorted(t, t[release]+0.35)
    assert np.all(origin[start:end] == origin[start])
    sl = slice(start, end)
    elapsed = t[sl]-t[release]
    displacement = np.linalg.norm(xy[sl]-xy[start], axis=1)
    speed = wheels[sl].mean(axis=1)
    travel = np.cumsum(np.r_[0., np.diff(t[sl])*(speed[1:]+speed[:-1])/2])
    last = release-start-1
    rows.append([name,last+1,1000*(t[release]-t[start]),displacement[:last+1].max(),
                 travel[last],displacement[last+1],displacement[-1]])
    ax.plot(elapsed,travel,"--",label="Integrated wheel speed")
    ax.plot(elapsed,displacement,"o-",ms=3,label="SDK XY distance")
    ax.axvline(0,color="gray",lw=1)
    ax.set_title("Session "+name)
    ax.set_xlabel("Seconds from first off-contact report")
    ax.set_ylabel("mm from first rolling report")
axes.flat[0].legend(fontsize=7)
fig.savefig(here/"departure-pose.png",dpi=160)
with (here/"departure-pose.csv").open("w", newline="") as stream:
    writer=csv.writer(stream,lineterminator="\n")
    writer.writerow(["session","rolling_docked_rows","first_rolling_to_release_ms","max_docked_xy_mm",
                     "integrated_docked_wheel_mm","first_off_xy_mm","last_xy_mm"])
    writer.writerows(rows)

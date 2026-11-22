"""Transfer two earlier robot-receipt predictors to host receipt during LED trials."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[1] / "unit3-cube-accelerometer"))
from packet_cadence import clocks
from receipt_transfer import predict


def intervals(times):
    delta = np.diff(times)
    gaps = np.flatnonzero(delta > .15)
    return delta, gaps


rest = clocks("resting")[0]
dt, gaps = intervals(rest)
normal = dt[dt <= .15].mean()
pause = dt[gaps].mean()
spacing = np.diff(gaps).mean()
period = np.polyfit(np.arange(len(gaps)), rest[gaps+1]-rest[gaps[0]+1], 1)[0]
with (here / "events.csv").open() as f:
    times = np.array([float(r["seconds"]) for r in csv.DictReader(f) if r["kind"] == "accel"])
times -= times[0]
delta, gaps = intervals(times)
index = np.arange(len(times))
forecast = predict(index, normal, pause, spacing)
restarts = times[gaps+1]
expected = restarts[0] + np.arange(len(gaps))*period
error_ms = 1000*(expected[1:]-restarts[1:])
fig, axes = plt.subplots(2, 1, figsize=(8, 6), layout="constrained")
axes[0].plot(times, 0.03*index-times, label="30 ms emission")
axes[0].plot(times, forecast-times, label="Frozen receiver prediction")
axes[0].axhline(0, color=".5", lw=.7)
axes[0].set(ylabel="Predicted − observed time (s)", xlabel="Host elapsed time (s)")
axes[0].legend()
axes[1].plot(times[1:], 1000*delta, ".", ms=3, label="Host interarrival")
axes[1].plot(expected[1:], 1000*np.full(len(expected)-1,np.median(dt[np.flatnonzero(dt>.15)])),
             "|", ms=15, label="Predicted restart after first anchor")
axes[1].legend()
axes[1].set(xlabel="Host elapsed time (s)", ylabel="Interval (ms)")
fig.savefig(here / "receiver-holdout.png", dpi=150)
print(len(times), len(gaps), forecast[-1], times[-1], 100*(forecast[-1]/times[-1]-1),
      np.median(abs(error_ms)), max(abs(error_ms)), np.round(delta[gaps],3))

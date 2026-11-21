"""Predict elapsed receiver time from packet count, without inferring sample loss."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from decode import decode

here = Path(__file__).resolve().parent
names = ["resting", "undock-check", "pickup"]


def clocks(name):
    packets, _ = decode(here / (name + ".tsv"))
    host = np.array([p[0] for p in packets], dtype=np.int64)
    uptime = np.array([p[1] for p in packets])
    return uptime - uptime[0], (host - host[0]) * 1e-9


def estimate(t):
    intervals = np.diff(t)
    # Halfway between the normal 30 ms arrivals and the distinct 250+ ms pauses.
    slow = intervals > .15
    return (np.median(intervals[~slow]), np.median(intervals[slow]),
            np.median(np.diff(np.flatnonzero(slow))))


def main():
    normal, pause, period = estimate(clocks("resting")[0])
    rows = []
    fig, axes = plt.subplots(2, 3, figsize=(12, 6))
    for j, name in enumerate(names):
        robot, pod = clocks(name)
        index = np.arange(len(robot))
        normal_clock = .03 * index
        # Start at phase zero for every recording. Never fit phase on a test file.
        proxy = normal * index + (pause-normal) * np.floor(index/period)
        ax = axes[0,j]
        ax.plot(index[1:], np.diff(pod)*1000, ".", ms=2, alpha=.55, label="Pod receipt")
        ax.plot(index[1:], np.diff(robot)*1000, ".", ms=2, label="Robot receipt")
        ax.set(title=name, ylabel="Interarrival (ms)")
        ax = axes[1,j]
        ax.plot(index, normal_clock-robot, label="Uniform 30 ms")
        ax.plot(index, proxy-robot, label="Rest-calibrated pauses")
        ax.axhline(0, color=".7",lw=.6)
        ax.set(xlabel="Received packet index",ylabel="Elapsed time error (s)")
        rows.append([name,len(index),sum(np.diff(robot)>.15),sum(np.diff(pod)>.15),
                     robot[-1],normal_clock[-1],proxy[-1],np.max(np.abs(proxy-robot)),
                     100*(proxy[-1]/robot[-1]-1)])
    for ax in axes[:,0]:
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(here/"packet-cadence.png",dpi=150)
    with (here/"packet-cadence.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["capture","packets","long_robot_gaps","long_pod_gaps","elapsed_s","uniform_s",
                    "pause_prediction_s","max_timeline_error_s","endpoint_percent_error"])
        w.writerows(rows)
    print(normal,pause,period)
    for row in rows:
        print(*row)


if __name__=="__main__":
    main()

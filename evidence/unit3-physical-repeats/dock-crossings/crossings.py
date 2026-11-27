"""Measure held returns through stationary fork crossings in the six shelf sessions."""
import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
BASE = HERE.parent


def read(session, name):
    with gzip.open(BASE / session / (name + ".csv.gz"), "rt") as stream:
        return list(csv.DictReader(stream))


def values(rows, name):
    return np.array([float(r[name]) if r[name] else np.nan for r in rows])


def bracketed(session):
    states, ranges = read(session, "robot"), read(session, "range")
    assert [r["receipt_s"] for r in states] == [r["receipt_s"] for r in ranges]
    t, h, x, y, yaw, origin, left, right = (values(states, k) for k in
        ("receipt_s", "lift_mm", "x_mm", "y_mm", "yaw_rad", "origin", "left_mmps", "right_mmps"))
    distance, quality = (values(ranges, k) for k in ("distance_mm", "signal_quality"))
    dock = (values(states, "status").astype(int) & 0x1000) != 0
    still = (abs(left) <= 1) & (abs(right) <= 1) & np.isfinite(distance)
    split = (np.diff(t) > .2) | (np.diff(origin) != 0) | (dock[1:] != dock[:-1])
    bounds = np.flatnonzero(np.r_[True, (still[1:] != still[:-1]) | split, True])
    for a, b in zip(bounds, bounds[1:]):
        if not still[a] or np.hypot(np.ptp(x[a:b]), np.ptp(y[a:b])) > 2:
            continue
        if np.ptp(np.unwrap(yaw[a:b])) > np.deg2rad(2):
            continue
        clear = (h[a:b] < 34) | (h[a:b] > 56)
        edges = np.r_[a, a + 1 + np.flatnonzero(clear[1:] != clear[:-1]), b]
        for k in range(1, len(edges)-2):
            i, j = edges[k:k+2]
            if clear[i-a]:
                continue
            before = np.arange(edges[k-1], i)
            after = np.arange(j, edges[k+2])
            before = before[(t[i]-t[before] <= 1) & (t[i]-t[before] >= .15)]
            after = after[(t[after]-t[j] <= 1) & (t[after]-t[j] >= .15)]
            if min(len(before), len(after)) < 3:
                continue
            pre, post = np.median(distance[before]), np.median(distance[after])
            if abs(pre-post) > 20 or min(pre, post) < 45:
                continue
            yield dock[i], t[i:j], h[i:j], distance[i:j], quality[i:j], distance[i-1], quality[i-1]


fig, axes = plt.subplots(2, 3, sharex=True, figsize=(10.5, 5), layout="constrained")
with (HERE / "crossings.csv").open("w", newline="") as f:
    writer = csv.writer(f, lineterminator="\n")
    writer.writerow(["session", "on_dock", "start_s", "reports", "same_as_entry", "fresh_mae_mm", "hold_mae_mm"])
    for session in "ABCDEF":
        for dock, ts, h, d, q, d0, q0 in bracketed(session):
            writer.writerow([session, int(dock), ts[0], len(ts), sum((d==d0) & (q==q0)),
                             np.mean(abs(d-30)), np.mean(abs(d-d0))])
            if dock and session in "DEF":
                col = "DEF".index(session)
                elapsed = ts - ts[0]
                axes[0,col].plot(elapsed, h, ".-")
                axes[1,col].plot(elapsed, d, ".-")
                axes[1,col].plot(elapsed, np.full(len(ts),d0), "--", color="#464646")
                axes[0,col].set_title("Session " + session)
for ax in axes[1]:
    ax.axhline(30, color="#bc4829", lw=1.3)
    ax.set(xlabel="Seconds since entering lift window", ylim=(20,140))
axes[0,0].set_ylabel("Lift height (mm)")
axes[1,0].set_ylabel("SDK distance (mm)")
fig.savefig(HERE / "crossings.png", dpi=160)

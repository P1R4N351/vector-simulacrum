"""Test the change in cube inclination from shelf rest across two real pickups."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from carry_hold_transfer import select

here = Path(__file__).resolve().parent
early = here.parent / "unit3-cube-accelerometer"
later = here.parent / "unit3-wheelstand/R5"


def load(path):
    return np.loadtxt(path, delimiter=",", skiprows=1)


def command(path, action, clock):
    with path.open() as stream:
        return float(next(row[clock] for row in csv.DictReader(stream)
                          if row["action"] == action and row.get("kind", row.get("event")) == "begin"))


def changes(samples, reference):
    return np.degrees(np.arctan2(np.linalg.norm(np.cross(samples, reference), axis=1),
                                 samples @ reference))


a = load(early / "robot.csv")
b = load(later / "state.csv.gz")
p = load(early / "pickup.csv")[:, [0, 2, 4, 5, 6]]
p[:, 0] /= 1e9
q = load(later / "cube.csv.gz")[:, [0, 2, 4, 5, 6]]
datasets = [
    ("Earlier pickup", p, np.column_stack([a[:, 0], a[:, 1], a[:, 3:5], a[:, 5] == 1]),
     command(early / "action-times.csv", "pickup", "pod_unix_s")),
    ("R5 pickup", q, np.column_stack([b[:, 0], b[:, 3:6], (b[:, 6].astype(int) & 2) != 0]),
     command(later / "actions.csv", "pickup for inward transport", "sec")),
]
fig, axes = plt.subplots(1, 2, sharey=True, figsize=(9, 4), layout="constrained")
results = []
for ax, (label, records, state, origin) in zip(axes, datasets):
    packets = records.reshape(-1, 3, 5)
    # Collapse immediate identical BLE notifications before estimating direction.
    keep = np.r_[True, np.any(np.diff(packets[:, :, 2:], axis=0) != 0, axis=(1, 2))]
    packets = packets[keep]
    time = packets[:, 0, 0] - origin
    rest = packets[(time >= -1) & (time < 0)]
    up = np.median(rest[:, :, 2:], axis=(0, 1))
    up /= np.linalg.norm(up)
    held, heights = select(state, packets.reshape(-1, 5))
    apparent = changes(np.mean(held[:, :, 2:], axis=1), up)
    mid = np.median(apparent)
    if not results:
        prediction = mid
    lo, hi = np.percentile(apparent, [5, 95])
    if label == "Earlier pickup":
        replay = np.genfromtxt(early / "free-pickup.csv", delimiter=",", names=True, dtype=None, encoding="utf8")
        replay = replay[replay["initialization"] == "Before command"]
        sim_time = replay["time_s"]
    else:
        replay = np.genfromtxt(later / "pickup-replay.csv", delimiter=",", names=True)
        sim_time = replay["sec"]
    held_time = held[:, 0, 0] - origin
    common = (held_time >= sim_time.min()) & (held_time <= sim_time.max())
    sim_angle = np.interp(held_time[common], sim_time, replay["tilt_deg"])
    ax.scatter(held_time[common], sim_angle, marker="x", s=10, label="Free-cube replay")
    results.append([label, len(rest), len(held), np.median(heights), lo, mid, hi, mid - prediction, common.sum(), np.median(abs(sim_angle - apparent[common]))])
    ax.scatter(held[:, 0, 0] - origin, apparent, s=7, label="Stopped, carrying")
    ax.axhline(prediction, ls="--", color="darkred", label="Earlier pickup median")
    ax.axhline(0, ls=":", color=".4", label="Original shelf attitude")
    ax.set(title=label, xlabel="Seconds from pickup command")
axes[0].set_ylabel("Change in gravity direction from shelf rest (°)")
axes[1].legend(fontsize=8)
fig.savefig(here / "carry-angle-transfer.png", dpi=160)
with (here / "carry-angle-transfer.csv").open("w") as stream:
    writer = csv.writer(stream)
    writer.writerow("recording rest_packets carry_packets median_lift_mm p05_deg median_deg p95_deg transfer_error_deg overlapping_packets replay_median_abs_error_deg".split())
    writer.writerows(results)

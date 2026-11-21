"""Plot from native SDK events. Time is host receipt relative to the pickup call."""

import gzip
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

here = Path(__file__).resolve().parent
with gzip.open(here / "events.jsonl.gz", "rt") as f:
    rows = [json.loads(s) for s in f]
begin = next(
    r["t"] for r in rows if r.get("action") == "pickup" and r["kind"] == "begin"
)
end = next(r["t"] for r in rows if r.get("action") == "pickup" and r["kind"] == "end")
placed = next(r["t"] for r in rows if r.get("action") == "place" and r["kind"] == "end")
states = [
    r
    for r in rows
    if r.get("name") == "robot_state" and begin - 1 <= r["t"] <= placed + 0.15
]
t = np.array([r["t"] - begin for r in states])
d = [r["data"] for r in states]
fig, ax = plt.subplots(4, 1, figsize=(9, 7), sharex=True)
ax[0].plot(t, [s["lift_height_mm"] for s in d], label="Lift")
carry = [i for i, s in enumerate(d) if s.get("carrying_object_id", -1) == 1]
ax[0].axvspan(
    t[carry[0]], t[carry[-1]], alpha=0.16, color="green", label="Carrying ID 1"
)
ax[0].legend()
ax[0].set_ylabel("Height (mm)")
for k in ("left", "right"):
    ax[1].plot(t, [s.get(k + "_wheel_speed_mmps", 0) for s in d], label=k)
ax[1].set_ylabel("Wheel (mm/s)")
ax[1].legend()
ax[2].plot(
    t,
    [s["prox_data"].get("distance_mm", 0) for s in d],
    color=".65",
    label="All returns",
)
valid = [i for i, s in enumerate(d) if s["prox_data"].get("found_object", False)]
ax[2].scatter(
    t[valid],
    [d[i]["prox_data"]["distance_mm"] for i in valid],
    s=5,
    label="Found object",
)
ax[2].set_ylabel("Range (mm)")
ax[2].legend()
ax[3].plot(t, [s["gyro"].get("z", 0) for s in d], label="Head gyro Z")
ax[3].set_ylabel("Gyro (rad/s)")
ax[3].set_xlabel("Time from pickup request (s)")
for a in ax:
    a.axvline(0, c=".3", ls=":")
    a.axvline(end - begin, c=".3", ls=":")
fig.tight_layout()
fig.savefig(here / "pickup.png", dpi=150)

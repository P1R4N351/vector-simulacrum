"""Compare unit 3 dock-contact transitions with frozen browser shelf physics."""
import csv
import gzip
import json
import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
RUNS = HERE.parent / "unit3-wheelstand"
# Repeated shelf departure imperative with an end response and both wheel channels.
TRIALS = "W2 W3 W4 W5 W8 R5 R7 R8 R9 R10 D2 R11".split()
inputs = {}
records = {}
for name in TRIALS:
    with (RUNS / name / "actions.csv").open() as f:
        actions = [x for x in csv.DictReader(f) if x["action"] == "undock"]
    begin, end = [float(next(x for x in actions if x["kind"] == kind)["sec"]) for kind in ("begin", "end")]
    with gzip.open(RUNS / name / "state.csv.gz", "rt") as f:
        rows = [x for x in csv.DictReader(f) if begin <= float(x["sec"]) <= end]
    a = np.array([[float(x["sec"])-begin, float(x["left"]), float(x["right"]), bool(int(x["status"]) & 0x1000), float(x["head"]), float(x["lift"]), float(x["pitch"])] for x in rows])
    i = np.flatnonzero(np.diff(a[:, 3]) == -1)[0] + 1
    a = a[a[:, 0] <= a[i, 0] + 1.5]
    mean_speed = np.mean(a[:, 1:3], axis=1)
    travel = np.r_[0, np.cumsum(np.diff(a[:, 0]) * (mean_speed[:-1] + mean_speed[1:]) / 2)]
    inputs[name] = a[:, [0,1,2,4,5]].tolist()
    records[name] = (a, travel, i)

def replay(outward):
    proc = subprocess.run(["node", str(HERE / "replay.mjs")], check=True, capture_output=True,
                          input=json.dumps({"trials": inputs, "outwardMm": outward}), text=True)
    return json.loads(proc.stdout)

sim = replay(0)
calibration = []
for name in ("W2", "W3", "W4"):
    _, travel, i = records[name]
    b = np.asarray(sim[name])
    release = b[np.flatnonzero(b[:, 2] == 0)[0], 1]
    calibration.append(release - np.mean(travel[i-1:i+1]))
outward = float(np.median(calibration))
shifted = replay(outward)
print(f"Candidate seat shift from W2–W4: {outward:.3f} mm outward")
fig, axes = plt.subplots(3, 4, figsize=(14, 8), sharex=True, sharey=True)
with (HERE / "release.csv").open("w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["trial", "last_on_s", "first_off_s", "last_on_travel_mm", "first_off_travel_mm",
                "sim_release_s", "sim_release_tread_mm", "sim_release_chassis_mm", "candidate_release_s", "candidate_release_tread_mm",
                "physical_initial_pitch_deg", "nominal_initial_pitch_deg", "candidate_initial_pitch_deg"])
    for ax, name in zip(axes.flat, TRIALS):
        a, travel, i = records[name]
        t = a[i, 0]
        b = np.asarray(sim[name])
        off = np.flatnonzero(b[:, 2] == 0)
        values = b[off[0], [0, 1, 3]] if off.size else [np.nan] * 3
        c = np.asarray(shifted[name])
        idx = np.flatnonzero(c[:, 2] == 0)
        candidate = c[idx[0], :2] if idx.size else [np.nan, np.nan]
        w.writerow([name, *np.round([a[i-1, 0], t, travel[i-1], travel[i], *values, *candidate,
                                     *np.rad2deg([np.median(a[a[:, 0]<.5, 6]), b[0,4], c[0,4]])], 4)])
        ax.plot(a[:, 0]-t, travel, ".-", ms=2, label="Unit 3 encoder integral")
        ax.plot(b[:, 0]-t, b[:, 1], label="MuJoCo tread travel")
        ax.axvspan(a[i-1, 0]-t, 0, color="tab:blue", alpha=.2, label="Unit 3 flag transition")
        if off.size:
            ax.axvline(values[0]-t, color="tab:orange", ls="--", label="Simulated pin release")
        if idx.size:
            ax.axvline(candidate[0]-t, color="tab:green", ls=":", label="Shallower seat release")
        ax.set_title(name + (" (fit)" if name in ("W2", "W3", "W4") else ""))
        ax.set_xlim(-.8, .9)
        ax.set_ylim(-4, 42)
        ax.grid(alpha=.2)
fig.supxlabel("Seconds relative to first SDK report off charger")
fig.supylabel("Mean signed tread travel since undock request (mm)")
fig.legend(*axes.flat[0].get_legend_handles_labels(), loc="upper center", ncol=3)
fig.tight_layout(rect=(.02,.025,1,.91))
fig.savefig(HERE / "release.png", dpi=150)

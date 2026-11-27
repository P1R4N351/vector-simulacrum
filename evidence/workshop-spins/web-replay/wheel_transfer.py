"""Steady-state wheel/yaw decomposition with a complete visit held out."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent


def read(path):
    with path.open() as src:
        return list(csv.DictReader(src))


def col(rows, key):
    return np.array([float(row[key]) for row in rows])


web = [r for r in read(ROOT / "browser.csv") if r["scene"] == "vector-free-dock"]
results = []
for session in (".", "speed-holdout", "interleaved"):
    folder = ROOT.parent / session
    states = read(folder / "states.csv")
    events = read(folder / "events.csv")
    spins = [r for r in events if r["event"] == "spin"]
    stops = [r for r in events if r["event"] == "stop"]
    for turn, (spin, stop) in enumerate(zip(spins, stops, strict=True), 1):
        start, end = float(spin["time_s"]), float(stop["time_s"])
        samples = [r for r in states if (start + end) / 2 <= float(r["time_s"]) < end]
        times = col(samples, "time_s") - start
        pred = [r for r in web if r["session"] == session and int(r["turn"]) == turn]
        angle = col(pred, "head_rad") + np.deg2rad(-.66)
        sim_yaw = np.interp(times, col(pred, "t_s"), col(pred, "gx") * np.sin(angle)
                            + col(pred, "gz") * np.cos(angle)).mean()
        sim_diff = np.interp(times, col(pred, "t_s"), col(pred, "right") - col(pred, "left")).mean()
        angle = col(samples, "head_rad") + np.deg2rad(-.66)
        yaw = (col(samples, "gyro_x") * np.sin(angle) + col(samples, "gyro_z") * np.cos(angle)).mean()
        diff = (col(samples, "right_mm_s") - col(samples, "left_mm_s")).mean()
        speed = int(spin["detail"]) * (100 if session == "." else 1)
        results.append(dict(session=session, turn=turn, speed=speed, measured=yaw,
                            wheel_difference=diff, default=sim_yaw,
                            wheel_normalized=sim_yaw * diff / sim_diff))

training = [r for r in results if r["session"] != "interleaved"]
held = [r for r in results if r["session"] == "interleaved"]
for row in held:
    for name, group in [("pooled", training), ("same_speed", [
            r for r in training if abs(r["speed"]) == abs(row["speed"])])]:
        gain = np.mean([r["measured"] / r["wheel_difference"] for r in group])
        row[name] = gain * row["wheel_difference"]

held.sort(key=lambda r: (abs(r["speed"]), -r["speed"]))
with (ROOT / "wheel-transfer.csv").open("w") as out:
    writer = csv.DictWriter(out, fieldnames=list(held[0]))
    writer.writeheader()
    writer.writerows(held)
fig, ax = plt.subplots(figsize=(9, 4), layout="constrained")
ax.axhspan(-5, 5, color="#edf4ed")
ax.axhline(0, color="gray", linewidth=.8)
for i, (key, label) in enumerate([("default", "Web default"), ("wheel_normalized", "Web at measured wheel difference"),
                                 ("pooled", "Earlier visits, pooled gain"), ("same_speed", "Earlier visits, same-speed gain")]):
    errors = np.array([100 * (r[key] / r["measured"] - 1) for r in held])
    print(key, np.round(errors, 2), "RMS", round(np.mean(errors ** 2) ** .5, 2))
    ax.scatter(np.arange(len(held)) + (i - 1.5) * .10, errors, label=label)
ax.set_xticks(range(len(held)), [f"{abs(r['speed'])}\n{'CCW' if r['speed'] > 0 else 'CW'}" for r in held])
ax.set(xlabel="Commanded wheel speed (mm/s) and turn direction", ylabel="Steady yaw error (%)")
ax.legend(fontsize=8, ncols=2)
fig.savefig(ROOT / "wheel-transfer.png", dpi=160)

# Both other robots have archived turns at 150 mm/s with published steady windows.
external = ROOT.parent.parent / "recorded-pose-inset"
gain150 = np.mean([r["measured"] / r["wheel_difference"] for r in training if abs(r["speed"]) == 150])
other = []


def transfer(unit, spin, samples, yaw_key, left, right):
    measured = col(samples, yaw_key).mean()
    predicted = gain150 * (col(samples, right) - col(samples, left)).mean()
    other.append(dict(unit=unit, spin=spin, measured=measured, predicted=predicted,
                      error_pct=100 * (predicted / measured - 1)))


c7 = read(external / "c7_traces.csv")
for s in read(external / "c7_scores.csv"):
    if s["source"] == "unit 1" and abs(float(s["command_right_mm_s"])) == 150:
        samples = [r for r in c7 if r["spin"] == s["spin"] and r["source"] == "unit 1"
                   and float(s["window_start_s"]) <= float(r["t_s"]) < float(s["window_end_s"])]
        transfer("unit 1", s["spin"], samples, "yaw_rad_s", "left_mm_s", "right_mm_s")
p2 = [r for r in read(external / "traces.csv") if r["series"] == "recorded" and .7 <= float(r["t_s"]) <= 1.4]
for spin in sorted({r["spin"] for r in p2}):
    transfer("unit 3", spin, [r for r in p2 if r["spin"] == spin], "yaw", "wl", "wr")
with (ROOT / "wheel-transfer-other-surfaces.csv").open("w") as out:
    writer = csv.DictWriter(out, fieldnames=list(other[0]))
    writer.writeheader()
    writer.writerows(other)
print("External 150 mm/s errors (%)", np.round([r["error_pct"] for r in other], 2))

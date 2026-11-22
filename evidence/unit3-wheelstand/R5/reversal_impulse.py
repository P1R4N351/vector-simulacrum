"""Integrate contact torques about the fixed tilt axis at physical frame 91."""
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from pickup_onset import action_time, read

here = Path(__file__).resolve().parent
lights = read(here / "raise-lights.csv")
frame = float(lights["receipt_s"][lights["frame"] == 91].item())
frame -= action_time(here / "actions.csv", "pickup for inward transport", "sec")
inertia = 1.6133333333333334e-5  # isotropic cube fixture, kg m²
fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 6), layout="constrained")
rows = []
for path in sys.argv[1:]:
    a = np.loadtxt(Path(path) / "world-moment.csv", delimiter=",", skiprows=1)
    t = a[:, 0]
    dt = np.median(np.diff(t))
    start, end = [np.argmin(abs(t - x)) for x in (2.559, frame)]
    axis = a[end, 2:5] / np.linalg.norm(a[end, 2:5])
    speed = a[:, 5:8] @ axis
    moment = a[:, 8:].reshape(-1, 3, 3) @ axis
    airborne = start + 1 + np.flatnonzero(abs(moment[start:end, 0]) > 1e-12)[-1]
    for phase, i, j in (("shelf contact", start, airborne), ("after shelf release", airborne, end)):
        impulse = moment[i:j].sum(axis=0) * dt * 1000
        change = (speed[j] - speed[i]) * inertia * 1000
        np.testing.assert_allclose(impulse.sum(), change, atol=1e-8)
        rows.append([dt*1000, phase, t[i], t[j], *np.degrees(speed[[i,j]]), *impulse, change])
    span = slice(start, end+1)
    label = f"{dt*1000:.3g} ms step"
    axes[0].plot(t[span]-frame, np.degrees(speed[span]), label=label)
    axes[1].plot(t[span]-frame, (speed[span]-speed[airborne])*inertia*1000, label=label)
    for ax in axes:
        ax.axvline(t[airborne]-frame, lw=.9, ls=":", color=ax.lines[-1].get_color())
for ax in axes:
    ax.axhline(0, lw=.6, color=".5")
    ax.legend()
axes[0].set(title="R5 simulated pickup · dotted lines mark final shelf release", ylabel="Fixed-axis speed (°/s)")
axes[1].set(ylabel="Angular impulse since release (mN·m·s)", xlabel="Seconds relative to the 38.6 mm wrist frame")
fig.savefig(here / "reversal-impulse.png", dpi=160)
with (here / "reversal-impulse.csv").open("w") as stream:
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow("step_ms phase start_s end_s initial_deg_s final_deg_s shelf_mNms fork_mNms links_mNms momentum_change_mNms".split())
    writer.writerows(rows)

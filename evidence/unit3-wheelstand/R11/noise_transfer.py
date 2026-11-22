"""Compare no-contact cube acceleration in physical packets and the frozen sensor."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2]))
from simulacrum import params
from simulacrum.cube import CubeAccelerometer


def physical(run, action):
    folder = HERE.parent / run
    with (folder / "actions.csv").open() as stream:
        start, end = [float(r["sec"]) for r in csv.DictReader(stream) if r["action"] == action]
    data = np.genfromtxt(folder / "cube.csv.gz", delimiter=",", names=True)
    rows = data[(data["sec"] >= start) & (data["sec"] <= end)]
    slots = rows["slot"].reshape(-1, 3)
    assert np.all(slots == [0, 1, 2])
    return np.stack([rows[k] for k in "xyz"], axis=-1).reshape(-1, 3, 3) / 8192


def summarize(slots):
    magnitude = np.linalg.norm(slots, axis=-1)
    noise = 1000 * np.sqrt(np.mean(np.diff(slots, axis=1)**2, axis=(0, 1)) / 2)
    return [magnitude.mean(), 1000 * magnitude.std(), *noise]


def main():
    readings = np.genfromtxt(HERE.parents[1] / "unit3-cube-accelerometer/resting.csv", names=True, delimiter=",")
    gain = np.linalg.norm([readings[k + "_counts"].mean() for k in "xyz"]) / 8192
    config = params.values(params.load(model="vector_2"))
    config.update(cube_accel_scale=[gain]*3)
    windows = [("R10", "passive post-stop capture"), ("R11", "passive post-stop capture"), ("R11", "roll cube")]
    fig, ax = plt.subplots(figsize=(8, 4), layout="constrained")
    with (HERE / "noise-transfer.csv").open("w") as output:
        writer = csv.writer(output, lineterminator="\n")
        writer.writerow(["run", "window", "source", "packets", "mean_g", "norm_sd_mg", "x_diff_mg", "y_diff_mg", "z_diff_mg"])
        for i, (run, action) in enumerate(windows):
            observed = physical(run, action)
            chip = CubeAccelerometer(config, 0.0005, seed=i)
            force = np.array([0., 0., 1.])
            chip.reset(force, 0.)
            for step in range(1, 100000):
                chip.step(step * .0005, force)
                if len(chip.messages) >= len(observed) + 5:
                    break
            model = np.stack([np.frombuffer(msg[2:], "<i2").reshape(3, 3) for _, msg in list(chip.messages)[5:]]) / 8192
            assert len(model) == len(observed)
            for j, (source, slots) in enumerate([("Recorded", observed), ("Simulated", model)]):
                values = summarize(slots)
                writer.writerow([run, action, source, len(slots), *values])
                ax.bar(i + np.arange(3)*.22 + j*.09, values[2:], .085, color=["#236a83", "#da843e"][j], label=source if i==0 else None)
    ax.set_xticks([i+k*.22+.045 for i in range(3) for k in range(3)], list("XYZ")*3)
    for i, label in enumerate(["R10 at rest", "R11 at rest", "R11 roll miss"]):
        ax.text(i+.26, -.12, label, ha="center", transform=ax.get_xaxis_transform())
    ax.set_ylabel("Within-packet noise (mg RMS)")
    ax.legend()
    fig.savefig(HERE / "noise-transfer.png", dpi=150)


if __name__ == "__main__":
    main()

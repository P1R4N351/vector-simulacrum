"""Plot friction sweeps and held-out camera and gyro observations."""

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent / "evidence"
data = root / "lateral-friction"
plots = root / "physics-plots"


def read(file):
    with file.open() as f:
        return list(csv.DictReader(f))


def take(rows, **fields):
    return [r for r in rows if all(r[k] == str(v) for k, v in fields.items())]


colors = ["#14758c", "#c75d28"]
models = ["Cylinder", "Sphere"]
fig, axes = plt.subplots(2, 2, figsize=(10, 7), sharex=True)
for prefix, style, gravity in [("", "-", "B inclination"), ("level-", "--", "Level")]:
    scores = read(data / (prefix + "scores.csv"))
    for i, model in enumerate(models):
        rows = [
            r for r in take(scores, spot="B", model=model) if float(r["lateral_mu"]) > 0
        ]
        for ax, key in zip(axes[i], ["pair_rmse_mm", "gyro_rmse_dps"]):
            ax.plot(
                [float(r["lateral_mu"]) for r in rows],
                [float(r[key]) for r in rows],
                style,
                marker=".",
                color=colors[i],
                label=gravity,
            )
            ax.grid(alpha=0.2)
for i, model in enumerate(models):
    for j, ax in enumerate(axes[i]):
        ax.axvline(0.9, color=".55", lw=1)
        ax.set_title(model + " · " + ["paired drift RMSE (mm)", "gyro RMSE (°/s)"][j])
        if i == 1:
            ax.set_xlabel("Lateral friction coefficient")
axes[0, 0].legend()
fig.suptitle("Spot B · lateral friction sweep at 42 mm")
fig.tight_layout()
fig.savefig(plots / "lateral-friction-sweep.png", dpi=155)
plt.close(fig)

cam = read(root / "shelf-tilt" / "vectors.csv")
ends = read(data / "endpoints.csv")
gyro = read(data / "gyro.csv")
fig, axes = plt.subplots(2, 2, figsize=(9, 8), sharex=True, sharey=True)
for i, model in enumerate(models):
    for j, block in enumerate(["A0", "A180"]):
        ax = axes[i, j]
        for source, mu, label, color in [
            (cam, None, "Camera", "#202020"),
            (ends, 0.9, "μ = 0.90", colors[0]),
            (ends, 0.8875, "μ = 0.8875", colors[1]),
        ]:
            rows = take(source, model=model, block=block)
            if mu:
                rows = take(rows, lateral_mu=mu)
            keys = ["right_mm", "dockward_mm"] if mu else ["camera_x", "camera_y"]
            xy = np.cumsum(
                [[0, 0]]
                + [
                    [float(take(rows, spin=d)[0][k]) for k in keys]
                    for d in ["CCW", "CW"]
                ],
                axis=0,
            )
            ax.plot(*xy.T, marker="o", label=label, color=color)
            ax.scatter(*xy[-1], marker="s", color=color, zorder=4)
        ax.set(
            aspect="equal",
            title=f"{model} · {block}",
            xlabel="Shelf right (mm)",
            ylabel="Toward dock (mm)",
        )
        ax.grid(alpha=0.25)
axes[0, 0].legend(fontsize=8)
fig.suptitle("Held-out A · start, after CCW, after CW (square)")
fig.tight_layout()
fig.savefig(plots / "lateral-friction-transfer.png", dpi=155)
plt.close(fig)

fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True, sharey=True)
for i, block in enumerate(["A0", "A180"]):
    for j, spin in enumerate(["CCW", "CW"]):
        ax = axes[i, j]
        for m, model in enumerate(models):
            for mu, style in [(0.9, ":"), (0.8875, "-")]:
                rows = take(gyro, block=block, spin=spin, model=model, lateral_mu=mu)
                t = [float(r["seconds"]) for r in rows]
                ax.plot(
                    t,
                    [float(r["predicted_dps"]) for r in rows],
                    style,
                    color=colors[m],
                    label=f"{model} {mu}",
                    lw=1.6,
                )
        ax.plot(
            t,
            [float(r["observed_dps"]) for r in rows],
            "o-",
            ms=2,
            color="#262626",
            label="P2Y8",
        )
        ax.set(
            title=f"{block} · {spin}",
            xlabel="Replay time (s)",
            ylabel="Head gyro Z (°/s)",
        )
        ax.grid(alpha=0.2)
axes[0, 0].legend(fontsize=8)
fig.suptitle("Held-out A · measured and simulated head gyro, fixed B inclination")
fig.tight_layout()
fig.savefig(plots / "lateral-friction-gyro.png", dpi=155)

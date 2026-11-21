"""Publish the pressure experiment table and its principal comparisons."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from replay_pivot import DATA, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    rows = []
    for f in sorted(args.directory.glob("*.csv")):
        rows.extend(csv.DictReader(f.open()))

    def mean(shape, step, angle, mode, metric):
        values = [
            float(r["value"])
            for r in rows
            if r["geometry"] == shape
            and float(r["step_ms"]) == step
            and float(r["table_deg"]) == angle
            and r["input"] == mode
            and r["metric"] == metric
            and (mode not in ("balanced", "measured") or int(r["turn"]) >= 5)
        ]
        return np.mean(values) if values else np.nan

    with (DATA / "pressure-sweep.csv").open("w") as stream:
        w = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.5), layout="constrained")
    colors = {
        "stock": "#af543e",
        "narrow": "#99812a",
        "sphere": "#117c70",
        "paired": "#68598d",
    }
    labels = {
        "stock": "Full cylinder",
        "narrow": "Half-width cylinder",
        "sphere": "Centered sphere",
        "paired": "Paired spheres",
    }
    for shape, color in colors.items():
        angles = [0, 30, 60]
        a = axes[0, 0]
        a.plot(
            angles,
            [
                mean(shape, 1, i, "constant", "same_heading_migration_mm")
                for i in angles
            ],
            "o-",
            color=color,
            label=labels[shape],
        )
        steps = [1, 0.5, 0.25, 0.125]
        for ax, mode, metric in (
            (axes[0, 1], "balanced", "drift_mm"),
            (axes[1, 0], "measured", "gyro_rmse_dps"),
            (axes[1, 1], "native", "hybrid_brake_rmse_dps"),
        ):
            ax.plot(
                range(len(steps)),
                [mean(shape, v, 0, mode, metric) for v in steps],
                "o-",
                color=color,
            )
            ax.set(
                xticks=range(len(steps)),
                xticklabels=steps,
                xlabel="Integration step (ms)",
            )
    axes[0, 0].set(
        xlabel="Orientation of level table (degrees)",
        ylabel="Migration per revolution (mm)",
        title="Constant ±240 mm/s · same-heading positions",
        xticks=angles,
    )
    axes[0, 0].legend(fontsize=8)
    axes[0, 1].set(
        title="Endurance turns 5–9 · balanced inputs",
        ylabel="Rest-to-rest displacement (mm)",
    )
    axes[1, 0].set(
        title="Endurance turns 5–9 · measured wheel speeds",
        ylabel="Mean steady gyro RMSE (°/s)",
    )
    axes[1, 1].set(
        title="Native stops · both directions, hybrid inputs",
        ylabel="Mean braking gyro RMSE (°/s)",
    )
    for ax in axes.flat:
        ax.grid(alpha=0.2)
    fig.savefig(ROOT / "evidence/physics-plots/pressure-sweep.png", dpi=160)


if __name__ == "__main__":
    main()

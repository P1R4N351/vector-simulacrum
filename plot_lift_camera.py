"""Register the settled camera and replay endpoints in the dock frame."""

import csv

import matplotlib.pyplot as plt
import numpy as np

from replay_camera_spins import WORLD, calibration, intersect, project
from replay_lift_camera import DATA

GLYPH = np.array([[440, 88], [505, 89], [505, 150], [440, 149], [465, 260]]) + [
    390,
    455,
]
SHELF = np.array([[218, 163], [731, 166], [70, 265], [883, 293]]) + [390, 455]
OFFSET = [560, 580]  # Origin of the upright landmark crop in the full camera frame.
NAMES = ["Down A CCW", "Down A CW", "Up CCW", "Up CW", "Down B CCW", "Down B CW"]


def reconstruct(p, picks, height=65.5, eye_height=56.0, lever=17.3, back_drop=1.0):
    def xy(i, z):
        return intersect(picks[:, i : i + 2] + OFFSET, z, p)

    mark, back = xy(0, height), xy(2, height - back_drop)
    eye1, eye2 = xy(4, eye_height), xy(6, eye_height)
    span = eye2 - eye1
    front = np.column_stack((span[:, 1], -span[:, 0]))
    eyes = (eye1 + eye2) / 2
    has_eyes = np.isfinite(eyes[:, 0])
    heading = np.where(has_eyes[:, None], front, mark - back)
    heading /= np.linalg.norm(heading, axis=1)[:, None]
    visible = has_eyes & np.isfinite(mark[:, 0])
    # Use first rest and average duplicate frames from the second visible pose.
    delta = np.sum((eyes[visible] - mark[visible]) * heading[visible], axis=1)
    spacing = (delta[0] + np.mean(delta[1:])) / 2
    mark = np.where(np.isfinite(mark), mark, eyes - spacing * heading)
    return mark + lever * heading, heading


def main():
    table = np.genfromtxt(DATA / "picks.csv", delimiter=",", skip_header=1)
    p = calibration(GLYPH, SHELF)
    cam, heading = reconstruct(p, table[:, 1:])
    vectors = {"Camera": cam[1::2] - cam[::2]}
    records = list(csv.DictReader((DATA / "replays.csv").read_text().splitlines()))
    for model in ("SDK", "Cylinder", "Sphere"):
        local = np.array(
            [
                [float(r["forward_mm"]), float(r["left_mm"])]
                for r in records
                if r["source"] == model
            ]
        )
        h = heading[::2]
        vectors[model] = local[:, :1] * h + local[:, 1:] * np.column_stack(
            (-h[:, 1], h[:, 0])
        )
    rng = np.random.default_rng(217)
    samples = []
    for _ in range(200):
        q = calibration(
            GLYPH + rng.uniform(-2, 2, (5, 2)),
            SHELF + rng.uniform(-6, 6, (4, 2)),
            rng.uniform(450, 720),
            rng.uniform(3, 12),
        )
        perturbed = table[:, 1:] + rng.uniform(-1.25, 1.25, (12, 8))
        xy, _ = reconstruct(
            q,
            perturbed,
            rng.uniform(63, 68),
            rng.uniform(50, 63),
            rng.uniform(11, 20),
            rng.uniform(0, 2),
        )
        samples.append(xy[1::2] - xy[::2])
    samples = np.array(samples)
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(12, 5.8), layout="constrained")
    colors = ["#17475c", "#797979", "#d66817", "#7564a6"]
    xx = np.arange(6)
    with (DATA / "by-turn.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["turn", "source", "right_mm", "away_mm", "distance_mm"])
        for k, ((name, v), color) in enumerate(zip(vectors.items(), colors)):
            track = np.vstack(([0, 0], np.cumsum(v, axis=0)))
            ax.plot(*track.T, "o-", label=name, color=color, ms=4)
            pos = xx + (k - 1.5) * 0.18
            bx.bar(pos, np.linalg.norm(v, axis=1), 0.17, color=color, label=name)
            for label, d in zip(NAMES, v):
                w.writerow([label, name, *d, np.linalg.norm(d)])
            if k == 0:
                for i, (x, y) in enumerate(track[1:], 1):
                    ax.annotate(
                        str(i), (x, y), xytext=(3, 4), textcoords="offset points"
                    )
                lo, hi = np.percentile(np.linalg.norm(samples, axis=2), [5, 95], axis=0)
                bx.vlines(pos, lo, hi, color="black", lw=1.2)
    ax.set(
        aspect="equal",
        xlabel="Shelf right (mm)",
        ylabel="Away from camera (mm)",
        title="Accumulated spin displacements",
    )
    bx.set(
        xticks=xx,
        xticklabels=[n.replace(" ", "\n", 1) for n in NAMES],
        ylabel="Rest-to-rest displacement (mm)",
        title="Each turn, in execution order",
    )
    for axis in (ax, bx):
        axis.grid(alpha=0.2)
        axis.set_axisbelow(True)
    ax.legend()
    fig.suptitle("Vector 2.0  ·  150 mm/s  ·  lift down → up → down")
    fig.savefig(DATA.parent / "physics-plots/lift-camera-drift.png", dpi=185)
    print(
        "Dock RMS per point",
        np.mean(np.sum((project(WORLD, p) - GLYPH) ** 2, axis=1)) ** 0.5,
    )
    print(
        "First-to-last", cam[-1] - cam[0], "between-spin gaps", cam[2::2] - cam[1:-1:2]
    )
    for model, v in vectors.items():
        print(
            model,
            "turns",
            np.linalg.norm(v, axis=1).round(1),
            "pairs",
            v.reshape(3, 2, 2).sum(axis=1).round(1),
            "sum",
            v.sum(axis=0).round(1),
        )
    print(
        "Sensitivity per turn",
        np.percentile(np.linalg.norm(samples, axis=2), [5, 95], axis=0).round(1),
    )
    print(
        "Pair sensitivity",
        np.percentile(
            np.linalg.norm(samples.reshape(-1, 3, 2, 2).sum(axis=2), axis=2),
            [5, 95],
            axis=0,
        ).round(1),
    )


if __name__ == "__main__":
    main()

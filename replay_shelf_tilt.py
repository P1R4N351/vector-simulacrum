"""Estimate the uphill gravity projection at B and replay it unchanged at A."""

import csv
import json

import matplotlib.pyplot as plt
import numpy as np

from plot_heading_crossover import BLOCKS, basis, optical
from replay_endurance_drift import relative_center
from replay_measured_turn import run
from replay_pivot import configure_round
from see3cam_shelf import DATA, camera_pose, edge_groups, lens_fit

OUT = DATA.parent / "shelf-tilt"
MODELS = ("Cylinder", "Sphere")


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def estimate(blocks, heads, rests):
    mats = []
    readings = []
    for block in blocks:
        rows = [r for r in rests if r["block"] == block]
        a = np.array([[float(r[k]) for k in ("ax", "ay", "az")] for r in rows])
        ang = np.array([float(r["head_rad"]) for r in rows])
        body = np.column_stack((a[:, 0] * np.cos(ang) - a[:, 2] * np.sin(ang), a[:, 1]))
        readings.append(np.mean(body, axis=0) / 9810)
        mats.append(basis(heads[block][0]))
    # Body-fixed offsets disappear in the opposite-heading difference.
    return np.linalg.solve(mats[0] - mats[1], readings[0] - readings[1])


def main():
    picks = np.loadtxt(
        DATA / "picks.csv", delimiter=",", skiprows=1, usecols=range(3, 7)
    )
    lens = lens_fit(edge_groups())
    _, h, *_ = optical(picks, camera_pose(lens), lens)
    heads = dict(zip(BLOCKS, h.reshape(4, 3, 2)))
    rests = read(OUT / "rest.csv")
    for r in rests:
        assert (
            float(r["wl"]) == float(r["wr"]) == 0
            and np.linalg.norm([float(r[k]) for k in ("gx", "gy", "gz")]) < 0.05
        )
    up = estimate(("B0", "B180"), heads, rests)
    for spot in "BA":
        u = estimate((spot + "0", spot + "180"), heads, rests)
        print(
            spot,
            "upward specific force, shelf x/y in g",
            u,
            "tilt degrees",
            np.degrees(np.arcsin(np.linalg.norm(u))),
        )
    reference = read(DATA / "vectors.csv")
    turns = json.loads((DATA / "turns.json").read_text())
    records = []
    for turn in turns:
        block, seq = turn["label"].split("_")
        stage = seq.upper()
        frame = basis(heads[block][int(seq == "cw")])
        rows = turn["rows"]
        t = np.array([r["t"] for r in rows])
        wheels = np.array([[r["wl"], r["wr"]] for r in rows])

        def baseline(source, block=block, stage=stage):
            r = next(
                r
                for r in reference
                if (r["block"], r["interval"], r["source"]) == (block, stage, source)
            )
            return np.array([float(r["shelf_right_mm"]), float(r["away_mm"])])

        actual = baseline("Camera")
        for model in MODELS:

            def configure(root, model=model, frame=frame):
                if model == "Sphere":
                    configure_round(root)
                opt = root.find("option")
                g = np.linalg.norm(np.fromstring(opt.get("gravity"), sep=" "))
                vec = -g * np.r_[frame @ up, np.sqrt(1 - up @ up)]
                opt.set("gravity", " ".join(map(str, vec)))

            _, out = run(
                rows,
                t,
                wheels,
                42,
                dense=True,
                include_pose=True,
                step_ms=1 if model == "Cylinder" else 0.125,
                configure=configure,
            )
            shifted = relative_center(out[:, 3:5], out[:, 5])[-1] @ frame
            records.append([block, stage, model, *actual, *baseline(model), *shifted])
    with (OUT / "vectors.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            [
                "block",
                "spin",
                "model",
                "camera_x",
                "camera_y",
                "flat_x",
                "flat_y",
                "tilt_x",
                "tilt_y",
            ]
        )
        w.writerows(records)
    fig, axes = plt.subplots(
        2, 2, figsize=(9, 7), layout="constrained", sharex=True, sharey=True
    )
    for ax, (block, model) in zip(
        axes.flat, [(b, m) for b in ("A0", "A180") for m in MODELS]
    ):
        data = np.array(
            [r[3:] for r in records if r[0] == block and r[2] == model]
        ).reshape(2, 3, 2)
        for vec, label, color in zip(
            data.transpose(1, 0, 2),
            ["Camera", "Flat", "Measured tilt"],
            ["#175c72", "#888888", "#ca6516"],
        ):
            path = np.vstack(([0, 0], np.cumsum(vec, axis=0)))
            ax.plot(*path.T, "o-", label=label, color=color)
        ax.set_aspect("equal")
        ax.set_title(f"{model} · A {block[1:]}°")
        ax.grid(alpha=0.25)
    axes[0, 0].legend()
    fig.supxlabel("Shelf right (mm)")
    fig.supylabel("Toward dock (mm)")
    fig.suptitle("Held-out spot A · start → CCW stop → CW stop")
    fig.savefig(DATA.parent / "physics-plots/shelf-tilt-heldout.png", dpi=180)
    fig2, (slope_ax, drift_ax) = plt.subplots(
        1, 2, figsize=(9, 4.2), layout="constrained"
    )
    for spot, color in [("B", "#175c72"), ("A", "#ca6516")]:
        downhill = -estimate((spot + "0", spot + "180"), heads, rests) * 100
        slope_ax.plot(
            [0, downhill[0]],
            [0, downhill[1]],
            "o-",
            color=color,
            label="B fit" if spot == "B" else "A independent check",
        )
    slope_ax.set(
        aspect="equal",
        xlabel="Rightward grade (%)",
        ylabel="Dockward grade (%)",
        title="Downhill direction",
        xlim=(-1.65, 0.12),
        ylim=(-0.12, 1.25),
    )
    slope_ax.legend()
    angles = ["A0", "A180"]
    for col, (source, color) in enumerate(
        [
            ("Camera", "#175c72"),
            ("Cylinder flat", "#bdbdbd"),
            ("Cylinder tilt", "#bf6216"),
            ("Sphere flat", "#858585"),
            ("Sphere tilt", "#845187"),
        ]
    ):
        model = "Cylinder" if source.startswith(("Cylinder", "Camera")) else "Sphere"
        index = 3 if source == "Camera" else 5 if source.endswith("flat") else 7
        left = [
            -sum(r[index] for r in records if r[0] == b and r[2] == model)
            for b in angles
        ]
        drift_ax.bar(
            np.arange(2) + (col - 2) * 0.16, left, 0.15, label=source, color=color
        )
    drift_ax.set(
        xticks=[0, 1],
        xticklabels=["A 0°", "A 180°"],
        ylabel="Pair displacement to shelf-left (mm)",
        title="Held-out CCW + CW pairs",
    )
    drift_ax.axhline(0, color="#777", lw=0.7)
    drift_ax.legend(fontsize=8)
    for axis in (slope_ax, drift_ax):
        axis.grid(alpha=0.17)
        axis.set_axisbelow(True)
    fig2.savefig(DATA.parent / "physics-plots/shelf-tilt-contribution.png", dpi=180)
    for model in MODELS:
        a = np.array(
            [r[3:] for r in records if r[0].startswith("A") and r[2] == model]
        ).reshape(2, 2, 3, 2)
        pair = a.sum(axis=1)
        print(model, "pair vectors camera/flat/tilt", pair.round(2))
        for data in (a.reshape(4, 3, 2), pair):
            err = np.sqrt(
                np.mean(np.linalg.norm(data[:, 1:] - data[:, :1], axis=2) ** 2, axis=0)
            )
            print("flat/tilt vector RMSE", err)
        print(
            "fraction of missing leftward drift recovered",
            (pair[:, 2, 0] - pair[:, 1, 0]).mean()
            / (pair[:, 0, 0] - pair[:, 1, 0]).mean(),
        )


if __name__ == "__main__":
    main()

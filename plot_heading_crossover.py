"""Compare eight balanced spins in the shelf frame and in each pair's starting body frame."""

import argparse
import csv

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares

from replay_lift_camera import main as replay
from see3cam_shelf import DATA, GLYPH, SHELF, camera_pose, edge_groups, lens_fit, locate

BLOCKS = ["B0", "B180", "A180", "A0"]
COLORS = {
    "Camera": "#115674",
    "SDK": "#777777",
    "Cylinder": "#d3670b",
    "Sphere": "#8265a9",
}


def optical(picks, pose, lens, lens_height=65.5, lever=11.3, eye_noise=None):
    mark = locate(picks[:, :2], lens_height, pose, lens)

    def lengths(z):
        return np.linalg.norm(locate(picks[:, 2:], z, pose, lens) - mark, axis=1)

    # The leveled vent and fixed indicator are rigid. Estimate their relative
    # height from all orientations, with no odometry or gyro in the fit.
    z, spacing = least_squares(
        lambda q: lengths(q[0]) - q[1],
        [65, 19],
        bounds=([48, 8], [78, 40]),
        loss="soft_l1",
    ).x
    vent = locate(picks[:, 2:], z, pose, lens)
    heading = (vent - mark) / lengths(z)[:, None]
    # Front views also expose the transverse eye axis at a common height.
    # This gives a longer, independent baseline across the image.
    with (DATA / "eye-check.csv").open() as f:
        for j, row in enumerate(csv.DictReader(f)):
            uv = np.array(
                [
                    [float(row[key + "_u"]), float(row[key + "_v"])]
                    for key in ("left", "right")
                ]
            )
            if eye_noise is not None:
                uv += eye_noise[j]
            left, right = locate(uv, 56.0, pose, lens)
            width = right - left
            heading[3 * BLOCKS.index(row["block"]) + int(row["rest"])] = [
                width[1],
                -width[0],
            ] / np.linalg.norm(width)
    return (
        mark + lever * heading,
        heading,
        vent - (spacing - lever) * heading,
        z,
        spacing,
    )


def basis(h):
    return np.array([[h[0], h[1]], [-h[1], h[0]]])


def compare(v, h1, h2):
    # Compare the reversed-heading observation to the first vector kept
    # fixed to the shelf, or rotated through the actual change of heading.
    a, b = v
    rotate = basis(h1).T @ basis(h2)
    return np.linalg.norm(b - a), np.linalg.norm(b - a @ rotate)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", action="store_true")
    if parser.parse_args().replay:
        replay(DATA)
    picks = np.loadtxt(
        DATA / "picks.csv", delimiter=",", skiprows=1, usecols=range(3, 7)
    )
    groups = edge_groups()
    lens = lens_fit(groups)
    pose = camera_pose(lens)
    xy, h, vent, z, spacing = optical(picks, pose, lens)
    print("Vent height and spacing", z, spacing)
    pts = xy.reshape(4, 3, 2)
    heads = h.reshape(4, 3, 2)
    drift = {"Camera": np.diff(pts, axis=1)}
    rows = list(csv.DictReader((DATA / "replays.csv").read_text().splitlines()))
    for model in ("SDK", "Cylinder", "Sphere"):
        local = [
            (float(r["forward_mm"]), float(r["left_mm"]))
            for r in rows
            if r["source"] == model
        ]
        drift[model] = np.array(
            [v @ basis(head) for v, head in zip(local, heads[:, :2].reshape(-1, 2))]
        ).reshape(4, 2, 2)
    with (DATA / "vectors.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            [
                "block",
                "interval",
                "source",
                "shelf_right_mm",
                "away_mm",
                "initial_forward_mm",
                "initial_left_mm",
            ]
        )
        for label, v in drift.items():
            for block, block_name in enumerate(BLOCKS):
                for d, stage in zip(
                    [*v[block], v[block].sum(axis=0)], ["CCW", "CW", "pair"]
                ):
                    w.writerow(
                        [block_name, stage, label, *d, *(d @ basis(heads[block, 0]).T)]
                    )
    rng = np.random.default_rng(117)
    samples = []
    lenses = [lens_fit(groups, ids) for ids in [(0, 1, 2), (0, 1, 3), (0, 1, 2, 3)]]
    for k in range(180):
        ln = lenses[k % len(lenses)]
        pp = camera_pose(
            ln,
            GLYPH + rng.uniform(-1.5, 1.5, (5, 2)),
            SHELF + rng.uniform(-5, 5, (4, 2)),
            rng.uniform(450, 720),
            rng.uniform(3, 12),
        )
        cp, ch, *_ = optical(
            picks + rng.uniform(-3, 3, picks.shape),
            pp,
            ln,
            rng.uniform(63, 68),
            rng.uniform(8, 14),
            rng.uniform(-1, 1, (4, 2, 2)),
        )
        v = cp[2::3] - cp[::3]
        hs = ch[::3]
        samples.append(
            [
                v[:, 0].tolist(),
                [*compare(v[[0, 1]], *hs[[0, 1]]), *compare(v[[3, 2]], *hs[[3, 2]])],
            ]
        )
    print(
        "shelf x sensitivity 5/95",
        np.percentile(np.array(samples)[:, 0], [5, 95], axis=0).round(1),
    )
    print(
        "mismatch sensitivity 5/95 B shelf/body A shelf/body",
        np.percentile(np.array(samples)[:, 1], [5, 95], axis=0).round(1),
    )
    head_angles = np.degrees(np.arctan2(heads[:, 0, 1], heads[:, 0, 0]))
    print(
        "headings",
        head_angles.round(1),
        "positions",
        pts[:, 0].round(1),
        "most rearward rest",
        pts[:, :, 1].max(),
    )
    print("vent-only pairs", (vent[2::3] - vent[::3]).round(1))
    fig, axes = plt.subplots(
        2, 2, figsize=(10, 7.8), sharex=True, sharey=True, layout="constrained"
    )
    for ax, k in zip(axes.flat, [3, 2, 0, 1]):
        for source, v in drift.items():
            track = np.vstack(([0.0, 0.0], np.cumsum(v[k], axis=0)))
            ax.plot(*track.T, "o-", color=COLORS[source], label=source, ms=4)
        ax.set(
            title=f"Spot {BLOCKS[k][0]}   ·   {BLOCKS[k][1:]}° start", aspect="equal"
        )
        ax.axhline(0, color="#dddddd", lw=0.6)
        ax.axvline(0, color="#dddddd", lw=0.6)
        ax.grid(alpha=0.22)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=4, loc="outside lower center")
    for ax in axes[-1]:
        ax.set_xlabel("Shelf right (mm)")
    fig.supylabel("Away from camera (mm)")
    fig.suptitle("Lift down, 42 mm replay  ·  start → CCW stop → CW stop")
    fig.savefig(DATA.parent / "physics-plots/heading-crossover-turns.png", dpi=180)
    plt.close(fig)
    fig, ax = plt.subplots(1, 2, figsize=(10, 4.8), layout="constrained")
    pair = drift["Camera"].sum(axis=1)
    for k, name in enumerate(BLOCKS):
        col = "#007484" if name.startswith("A") else "#984d0a"
        for a, vec in zip(ax, [pair[k], pair[k] @ basis(heads[k, 0]).T]):
            a.annotate(
                "",
                xy=vec,
                xytext=(0, 0),
                arrowprops={
                    "arrowstyle": "->",
                    "color": col,
                    "lw": 2,
                    "linestyle": "-" if name.endswith("180") else "--",
                },
            )
            offset = {
                "A0": (-14, 12),
                "B0": (-16, -15),
                "A180": (6, 6),
                "B180": (7, -10),
            }[name]
            a.annotate(
                name, xy=vec, xytext=offset, textcoords="offset points", color=col
            )
    for a in ax:
        a.set(aspect="equal", xlim=(-29, 29), ylim=(-29, 29))
        a.axhline(0, color="#ccc", lw=0.7)
        a.axvline(0, color="#ccc", lw=0.7)
        a.grid(alpha=0.2)
    ax[0].set(title="Shelf frame", xlabel="Right (mm)", ylabel="Away from camera (mm)")
    ax[1].set(
        title="Robot frame at start", xlabel="Forward (mm)", ylabel="Robot left (mm)"
    )
    fig.suptitle("The same four measured CCW/CW pair displacements")
    fig.savefig(DATA.parent / "physics-plots/heading-crossover-frames.png", dpi=180)
    plt.close(fig)
    for k, name in enumerate(BLOCKS):
        print(name, {s: v[k].sum(axis=0).round(1).tolist() for s, v in drift.items()})
    for i, j in [(3, 2), (0, 1)]:
        print(
            BLOCKS[i],
            "errors, shelf/body",
            compare(pair[[i, j]], heads[i, 0], heads[j, 0]),
            "start gap",
            np.linalg.norm(pts[i, 0] - pts[j, 0]),
        )


if __name__ == "__main__":
    main()

"""Separate head posture, arrival direction, and shelf-frame terms in static IMU windows."""

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from plot_heading_crossover import basis
from see3cam_shelf import camera_pose, edge_groups, lens_fit, locate

ROOT = Path(__file__).resolve().parent / "evidence"
DATA = ROOT / "head-sweep"
PLOTS = ROOT / "physics-plots"


def read(name):
    with (DATA / name).open() as f:
        return list(csv.DictReader(f))


def aggregate():
    raw = read("samples.csv")
    lens = lens_fit(edge_groups())
    pose = camera_pose(lens)
    marks = {}
    for r in read("marks.csv"):
        a, b = [
            locate([[float(r[k + "_u"]), float(r[k + "_v"])]], z, pose, lens)[0]
            for k, z in [("strip", 65.5), ("vent", 64.5)]
        ]
        h = (b - a) / np.linalg.norm(b - a)
        marks[r["block"]] = basis(h)
        print(
            r["block"],
            "center mm",
            np.round(a + 11.3 * h, 1),
            "heading",
            round(np.degrees(np.arctan2(*h[::-1])), 1),
        )
    windows = []
    for block, frame in marks.items():
        for visit in range(1, 7):
            rows = [r for r in raw if r["block"] == block and int(r["visit"]) == visit]
            selected = [
                r
                for r in rows
                if float(r["wl_mm_s"]) == float(r["wr_mm_s"]) == 0
                and np.linalg.norm([float(r[k + "_rad_s"]) for k in ("gx", "gy", "gz")])
                < 0.05
            ]
            assert len(selected) >= 10
            a = (
                np.array(
                    [
                        [float(r[k + "_mm_s2"]) for k in ("ax", "ay", "az")]
                        for r in selected
                    ]
                )
                / 9.81
            )
            head = np.array([float(r["head_deg"]) for r in selected])
            phi = np.radians(head)
            corrected = np.column_stack(
                (a[:, 0] * np.cos(phi) - a[:, 2] * np.sin(phi), a[:, 1])
            )
            windows.append(
                {
                    "block": block,
                    "angle": int(rows[0]["target_deg"]),
                    "head": head.mean(),
                    "visit": visit,
                    "arrival": 1 if block.endswith("ccw") else -1,
                    "frame": frame,
                    "raw": a[:, :2].mean(axis=0),
                    "xy": corrected.mean(axis=0),
                    "kept": len(selected),
                    "total": len(rows),
                }
            )
    return windows


def design(r, terms):
    cols = [np.eye(2)]
    if "head" in terms:
        cols.extend(np.eye(2) * (r["angle"] == angle) for angle in [-15, 10, 25, 40])
    if "shelf" in terms:
        cols.append(r["frame"])
    if "arrival" in terms:
        cols.append(np.eye(2) * r["arrival"])
    return np.hstack(cols)


def fit(rows, terms):
    X = np.vstack([design(r, terms) for r in rows])
    assert np.linalg.matrix_rank(X) == X.shape[1]
    return np.linalg.lstsq(X, np.concatenate([r["xy"] for r in rows]), rcond=None)[0]


def main():
    windows = aggregate()
    windows = [r for r in windows if r["block"] != "B0setup"]
    train = [r for r in windows if r["block"][0] == "B"]
    held = [r for r in windows if r["block"].startswith(("A0", "A180"))]
    across = [r for r in windows if r["block"].startswith("A90")]
    models = [
        (),
        ("head",),
        ("shelf",),
        ("arrival",),
        ("head", "shelf"),
        ("head", "arrival"),
        ("shelf", "arrival"),
        ("head", "shelf", "arrival"),
    ]
    stats = []
    for terms in models:
        coef = fit(train, terms)
        err = [
            np.sqrt(
                np.mean(
                    [np.sum((design(r, terms) @ coef - r["xy"]) ** 2) for r in group]
                )
            )
            for group in [train, held, across]
        ]
        stats.append([" + ".join(terms) or "Chassis only", *err])
    coef = fit(train, models[-1])
    print(
        "selected / total",
        sum(r["kept"] for r in windows),
        sum(r["total"] for r in windows),
    )
    print("B full coefficients in mg", coef.reshape(-1, 2).round(2))
    print("A independent diagnostic", fit(held, models[-1]).reshape(-1, 2).round(2))
    for row in stats:
        print(row[0], np.round(row[1:], 2))
    with (DATA / "transfer.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            [
                "terms_plus_chassis",
                "B_vector_rmse_mg",
                "A_vector_rmse_mg",
                "A90_vector_rmse_mg",
            ]
        )
        w.writerows(stats)
    trace_terms = ("head", "shelf")
    trace_coef = fit(train, trace_terms)
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    palette = {
        "0cw": "#307c99",
        "0ccw": "#64aacc",
        "180cw": "#9e4b21",
        "180ccw": "#df9258",
        "90ccw": "#75559e",
    }
    for j, spot in enumerate("BA"):
        for block in sorted({r["block"] for r in windows if r["block"][0] == spot}):
            rows = sorted(
                [r for r in windows if r["block"] == block], key=lambda r: r["head"]
            )
            for i in range(2):
                ax = axes[i, j]
                color = palette[block[1:]]
                ax.plot(
                    [r["head"] for r in rows],
                    [r["xy"][i] for r in rows],
                    "o",
                    color=color,
                    ms=4,
                    label=block[1:],
                )
                ax.plot(
                    [r["head"] for r in rows],
                    [(design(r, trace_terms) @ trace_coef)[i] for r in rows],
                    color=color,
                    lw=1.2,
                )
                ax.grid(alpha=0.18)
                ax.set_ylabel(["Forward", "Left"][i] + " specific force (mg)")
        axes[0, j].set_title(["B · fit", "A · held out"][j])
        axes[1, j].set_xlabel("Measured head angle (°)")
    axes[0, 1].legend(fontsize=8, ncol=2)
    fig.suptitle("Stationary head sweeps · measurements (dots) and B head + shelf-frame fit (lines)")
    fig.tight_layout()
    fig.savefig(PLOTS / "head-sweep-transfer.png", dpi=160)
    plt.close(fig)
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(12, 5))
    for spot, color in zip("BA", ["#307c99", "#c56c39"]):
        group = [r for r in windows if r["block"][0] == spot]
        ax.scatter(
            [r["head"] for r in group],
            [r["raw"][0] for r in group],
            s=16,
            facecolors="none",
            edgecolors=color,
            label=spot + " raw X",
        )
        ax.scatter(
            [r["head"] for r in group],
            [r["xy"][0] for r in group],
            s=10,
            color=color,
            label=spot + " corrected",
        )
    ax.set(
        xlabel="Measured head angle (°)",
        ylabel="Forward specific force (mg)",
        title="Gravity rotates through the head frame",
    )
    ax.legend(fontsize=8)
    ax.grid(alpha=0.2)
    names = [s[0] for s in stats]
    y = np.arange(len(names))
    for k, color in enumerate(["#307c99", "#c56c39", "#75559e"]):
        bx.barh(
            y + (k - 1) * 0.26,
            [s[1 + k] for s in stats],
            height=0.26,
            color=color,
            label=["B", "A opposite", "A sideways"][k],
        )
    bx.set_yticks(y, names)
    bx.invert_yaxis()
    bx.set_xlabel("Horizontal vector RMSE (mg)")
    bx.legend()
    bx.set_title("All terms fitted only on B")
    fig.tight_layout()
    fig.savefig(PLOTS / "head-sweep-components.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()

"""Reconstruct the external camera trajectory and replay both fixed Vector 2.0 contacts."""

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares, minimize_scalar
from scipy.spatial.transform import Rotation

from replay_endurance_drift import relative_center
from replay_measured_turn import run
from replay_pivot import configure_round

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "evidence/camera-spins"
FIG = ROOT / "evidence/physics-plots"
# Full-resolution pixels after rotation. World X is right, Y away from the
# camera, Z up. World origin lies on the shelf under the dock glyph.
WORLD = np.array(
    [[-23, 0, 71.5], [23, 0, 71.5], [23, 0, 25.5], [-23, 0, 25.5], [0, -94, 0]]
)
GLYPH_LIP = np.array(
    [[829, 542], [902, 543], [901, 611], [831, 611], [862, 717]], float
)
SHELF = np.array([[622, 610], [1116, 622], [456, 720], [1274, 746]], float)
PRINCIPAL = np.array([960, 540])


def project(xyz, pose):
    ray = Rotation.from_rotvec(pose[:3]).apply(xyz) + pose[3:6]
    xy = ray[:, :2] / ray[:, 2, None]
    h = np.linalg.norm(xy, axis=1)[:, None]
    return PRINCIPAL + pose[6] * xy * np.arctan(h) / np.maximum(h, 1e-9)


def intersect(uv, z, pose):
    xy = (uv - PRINCIPAL) / pose[6]
    h = np.linalg.norm(xy, axis=1)[:, None]
    ray = np.column_stack((xy * np.tan(h) / np.maximum(h, 1e-9), np.ones(len(h))))
    inv = Rotation.from_rotvec(pose[:3]).inv()
    center = inv.apply(-pose[3:6])
    direction = inv.apply(ray)
    return (center + direction * ((z - center[2]) / direction[:, 2])[:, None])[:, :2]


def calibration(pixels=GLYPH_LIP, shelf=SHELF, focal_prior=580, edge_weight=4):
    def residual(p):
        a, b, c, d = intersect(shelf, 0, p)
        across = (b - a) / np.linalg.norm(b - a)
        return np.r_[
            ((project(WORLD, p) - pixels) / 1.5).ravel(),
            (p[-1] - focal_prior) / 130,
            ((c - a) @ across) / edge_weight,
            ((d - b) @ across) / edge_weight,
        ]

    return least_squares(
        residual,
        [1.79, -0.08, 0.12, -62, 71, 389, 595],
        bounds=([-7] * 3 + [-2000] * 2 + [50, 300], [7] * 3 + [2000] * 3 + [1300]),
    ).x


def unit(v):
    return v / np.linalg.norm(v, axis=1)[:, None]


def reconstruct(p, uv, aim, eyes, rest, *, height=65.5, lever=11.3):
    mark = intersect(uv, height, p)
    heading = unit(intersect(aim, height, p) - mark)

    def eye_geometry(z):
        pair = intersect(eyes.reshape(-1, 2), z, p).reshape(-1, 2, 2)
        nose = pair.mean(axis=1) - mark[rest]
        span = pair[:, 1] - pair[:, 0]
        return pair, nose, span

    def loss(z):
        _, nose, span = eye_geometry(z)
        length = np.linalg.norm(nose, axis=1)
        skew = np.sum(span * unit(nose), axis=1)
        return np.sum((length - length.mean()) ** 2) + 2 * np.sum(skew**2)

    eye_z = minimize_scalar(loss, bounds=(48, 68), method="bounded").x
    pair, nose, span = eye_geometry(eye_z)
    front = unit(np.column_stack((span[:, 1], -span[:, 0])))
    for i, k in enumerate(rest):
        # The next command's pre-motion keyframe is in the same rest.
        heading[k] = front[i]
        if 0 < i < 4:
            heading[k + 1] = front[i]
    center = mark + lever * heading
    eye_center = (
        pair.mean(axis=1) - (np.mean(np.linalg.norm(nose, axis=1)) - lever) * front
    )
    return center - center[rest[0]], heading, eye_center - eye_center[0], eye_z


def main():
    picks = np.genfromtxt(DATA / "camera-picks.csv", names=True, delimiter=",")
    uv = np.column_stack((picks["u_px"], picks["v_px"]))
    aim = np.column_stack((picks["aim_u_px"], picks["aim_v_px"]))
    eyes = np.loadtxt(DATA / "eyes.csv", delimiter=",", skiprows=1)[:, 1:].reshape(
        5, 2, 2
    )
    # Robot SDK host led the camera capture host by about 18 ms.
    t = picks["time_s"] + 0.018
    rest = np.r_[0, np.flatnonzero(picks["key"] == 8)]
    p = calibration()
    camera, heading, eyes_only, eye_z = reconstruct(p, uv, aim, eyes, rest)
    # Convert SDK-local or simulation-local forward/left to dock X/Y using
    # only the initial heading from the camera. No per-turn registration.
    a, b = heading[0]
    orientation = np.array([[a, b], [-b, a]])
    rows = [
        r
        for r in map(json.loads, (DATA / "robot.jsonl").read_text().splitlines())
        if r["k"] == "s"
    ]
    time = np.array([r["t"] for r in rows])
    wheels = np.array([[r["wl"], r["wr"]] for r in rows])
    odom = (
        relative_center(
            np.array([r["pose"][:2] for r in rows]), np.array([r["yaw"] for r in rows])
        )
        @ orientation
    )
    traces = {"Camera": (t, camera), "SDK": (time, odom)}
    for name, step, shape in (
        ("42 mm cylinders", 1, None),
        ("42 mm spheres", 0.125, configure_round),
    ):
        clock, out = run(
            rows,
            time,
            wheels,
            42,
            dense=True,
            include_pose=True,
            step_ms=step,
            configure=shape,
        )
        path = relative_center(out[:, 3:5], out[:, 5]) @ orientation
        traces[name] = (clock, path)

    def sample(clock, xy, at):
        return np.column_stack([np.interp(at, clock, col) for col in xy.T])

    traces = {
        key: (clock, xy - sample(clock, xy, [t[0]])[0])
        for key, (clock, xy) in traces.items()
    }

    def lengths(c):
        return np.r_[
            np.linalg.norm(np.diff(c, axis=0), axis=1), np.linalg.norm(c[-1] - c[0])
        ]

    # Sensitivity ensemble, not a statistical confidence interval.
    rng = np.random.default_rng(71)
    sens = []
    for _ in range(160):
        pp = calibration(
            GLYPH_LIP + rng.uniform(-2, 2, (5, 2)),
            SHELF + rng.uniform(-6, 6, (4, 2)),
            rng.uniform(450, 720),
            rng.uniform(3, 12),
        )
        noisy = uv + rng.uniform(-2.5, 2.5, uv.shape)
        eye = eyes + rng.uniform(-1, 1, eyes.shape)
        c, _, _, _ = reconstruct(
            pp,
            noisy,
            aim,
            eye,
            rest,
            height=rng.uniform(62.5, 68.5),
            lever=rng.uniform(8.3, 14.3),
        )
        sens.append(lengths(c[rest]))
    ranges = np.percentile(sens, [5, 95], axis=0)
    print(
        "camera eye height",
        eye_z,
        "focal",
        p[-1],
        "RMS pixels",
        np.sqrt(np.mean(np.sum((project(WORLD, p) - GLYPH_LIP) ** 2, axis=1))),
    )
    print(
        "shelf rear width", np.linalg.norm(np.diff(intersect(SHELF[:2], 0, p), axis=0))
    )
    print("eye crosscheck", lengths(eyes_only).round(2), "sensitivity", ranges.round(2))
    end = {k: sample(ct, xy, t[rest]) for k, (ct, xy) in traces.items()}
    with (DATA / "optical-checks.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            [
                "interval",
                "primary_camera_mm",
                "eye_midpoint_check_mm",
                "sensitivity_p05_mm",
                "sensitivity_p95_mm",
            ]
        )
        for i, vals in enumerate(
            zip(lengths(camera[rest]), lengths(eyes_only), *ranges)
        ):
            w.writerow([i + 1 if i < 4 else "net", *np.round(vals, 3)])
    with (DATA / "by-turn.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            [
                "interval",
                "series",
                "dx_mm",
                "dy_mm",
                "displacement_mm",
                "error_vector_norm_mm",
            ]
        )
        for k, c in end.items():
            for i, delta in enumerate([*np.diff(c, axis=0), c[-1] - c[0]]):
                cam = (
                    np.diff(end["Camera"], axis=0)[i]
                    if i < 4
                    else end["Camera"][-1] - end["Camera"][0]
                )
                w.writerow(
                    [
                        i + 1 if i < 4 else "net",
                        k,
                        *np.round(delta, 3),
                        round(np.linalg.norm(delta), 3),
                        round(np.linalg.norm(delta - cam), 3),
                    ]
                )
    with (DATA / "aligned.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            ["t_s", *[k + "_" + axis + "_mm" for k in traces for axis in ("x", "y")]]
        )
        w.writerows(
            np.round(
                np.column_stack(
                    [t, *[sample(ct, xy, t) for ct, xy in traces.values()]]
                ),
                4,
            )
        )
    colors = ["#183047", "#bc6744", "#635a97", "#138171"]
    fig, (ax, bars) = plt.subplots(1, 2, figsize=(12, 5.5), layout="constrained")
    for j, (k, c) in enumerate(end.items()):
        ax.plot(*c.T, "o-", color=colors[j], label=k, lw=2, ms=4)
        x = np.arange(5) + (j - 1.5) * 0.19
        bars.bar(x, lengths(c), 0.18, color=colors[j], label=k)
        if j == 0:
            bars.vlines(
                x,
                ranges[0],
                ranges[1],
                color="#bd982b",
                lw=2,
                label="Camera sensitivity (5–95%)",
            )
            for i, point in enumerate(c):
                ax.annotate(str(i), point, xytext=(0, 7), textcoords="offset points")
    ax.set(
        title="Resting drive center after each turn",
        xlabel="Right along dock (mm)",
        ylabel="Away from camera (mm)",
        aspect="equal",
    )
    bars.set(
        title="Translation between rests and net displacement",
        ylabel="Distance (mm)",
        xticks=np.arange(5),
        xticklabels=["CCW 1", "CW 1", "CCW 2", "CW 2", "Net"],
    )
    bars.legend(fontsize=8)
    ax.grid(alpha=0.2)
    bars.grid(axis="y", alpha=0.2)
    fig.suptitle("Vector 2.0 · four alternating spins at ±150 mm/s")
    fig.savefig(FIG / "camera-rest-drift.png", dpi=160)
    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True, layout="constrained")
    for (k, (ct, xy)), color in zip(traces.items(), colors):
        for i, ax in enumerate(axes):
            ax.plot(
                ct,
                xy[:, i],
                ".-" if k == "Camera" else "-",
                ms=4,
                color=color,
                label=k,
                lw=1.2,
            )
    for ax, title in zip(axes, ("Right along dock", "Away from camera")):
        ax.set_ylabel(title + " (mm)")
        ax.grid(alpha=0.2)
        for i in rest[1:]:
            ax.axvline(t[i], color="#888", alpha=0.25)
    axes[0].legend(ncol=4, fontsize=8)
    axes[1].set_xlabel("Seconds after first wheel command")
    fig.suptitle("Camera keyframes, SDK odometry and continuous measured-wheel replays")
    fig.savefig(FIG / "camera-spin-traces.png", dpi=160)


if __name__ == "__main__":
    main()

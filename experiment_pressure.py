"""Controlled contact and timestep experiments on existing Vector 2.0 recordings."""

import argparse
import csv
import json
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
from functools import partial
from pathlib import Path

import numpy as np

from replay_endurance_drift import relative_center
from replay_hybrid_stops import switch_index
from replay_measured_turn import run
from replay_native_stops import COUNT_MM, LIFT_MM, delayed
from replay_native_stops import DATA as NATIVE
from replay_pivot import DATA, configure_plane, configure_round


def configure(root, shape, angle):
    if shape in ("sphere", "sphere-plane", "paired"):
        configure_round(root)
    if shape == "sphere-plane":
        configure_plane(root)
    table = root.find(".//geom[@name='table']")
    # MJCF compiler uses radians. Rotation about z leaves the tabletop level.
    theta = np.deg2rad(angle) / 2
    table.set("quat", f"{np.cos(theta)} 0 0 {np.sin(theta)}")
    for body in root.findall(".//body"):
        if not body.get("name", "").startswith("patch_"):
            continue
        g = body.find("geom")
        if shape == "narrow":
            g.set("fromto", "0 -0.0025 0 0 0.0025 0")
        elif shape == "paired":
            g.set("mass", str(float(g.get("mass")) / 2))
            g.set("pos", "0 -0.0025 0")
            other = deepcopy(g)
            other.set("pos", "0 0.0025 0")
            other.set("name", g.get("name") + "_second")
            body.append(other)


def run_case(case):
    shape, step, angle, mode = case
    conf = partial(configure, shape=shape, angle=angle)
    result = []

    def emit(turn, metric, value):
        result.append((shape, step, angle, mode, turn, metric, float(value)))

    if mode == "constant":
        # Identical smooth spin in each direction. Compare positions exactly
        # one revolution apart to cancel the origin offset and repeatable orbit.
        t = np.arange(0, 5.001, 0.005)
        speed = 240 * np.clip((t - 0.25) / 0.25, 0, 1)
        for label, sign in (("ccw", 1), ("cw", -1)):
            w = sign * np.column_stack((-speed, speed))
            clock, out = run(
                [{"head": 0, "lift": 32}],
                t,
                w,
                42,
                dense=True,
                include_pose=True,
                configure=conf,
                step_ms=step,
            )
            yaw = sign * np.unwrap(out[:, 5])
            moving = clock > 1
            phase = yaw[moving]
            if np.any(np.diff(phase) <= 0):
                raise ValueError("A steady revolution reversed")
            headings = np.arange(phase[0], phase[-1], 2 * np.pi)
            pts = np.column_stack(
                [np.interp(headings, phase, axis[moving]) for axis in out[:, 3:5].T]
            )
            for j, delta in enumerate(np.diff(pts, axis=0), 1):
                emit(label + str(j), "same_heading_migration_mm", np.linalg.norm(delta))
        return result
    if mode == "native":
        raw = list(csv.DictReader((NATIVE / "unit2-native-replay-input.csv").open()))
        for label in ("ccw_2", "cw_2"):
            group = [r for r in raw if r["turn"] == label]

            def col(k, group=group):
                return np.array([float(r[k]) for r in group])

            t = col("ms_from_last_edge") / 1000
            w = np.column_stack((col("left_mmps"), col("right_mmps")))
            counts = np.column_stack((col("left_count"), -col("right_count")))
            rate = COUNT_MM * np.diff(counts, axis=0) / np.diff(t)[:, None]
            i = switch_index(w)
            times = np.r_[t[: i + 1], t[i + 1 :]]
            inputs = np.vstack((w[: i + 1], rate[i:]))
            pose = [{"head": col("head_rad")[0], "lift": LIFT_MM}]
            for source, speeds in (("hybrid", inputs), ("reported", w)):
                clock, o = run(
                    pose, times, speeds, 42, dense=True, configure=conf, step_ms=step
                )
                error = delayed(clock, o[:, 0], t, 17) - np.degrees(col("head_gz"))
                for window, lo, hi in (("steady", -0.85, -0.25), ("brake", -0.1, 0.1)):
                    mask = (t >= lo) & (t <= hi)
                    emit(
                        label,
                        source + "_" + window + "_rmse_dps",
                        np.mean(error[mask] ** 2) ** 0.5,
                    )
                    if window == "steady" and source == "reported":
                        direction = 1 if label.startswith("ccw") else -1
                        emit(
                            label,
                            "steady_rate_bias_dps",
                            direction * np.mean(error[mask]),
                        )
        return result
    events = list(map(json.loads, (DATA / "run-g.jsonl").read_text().splitlines()))
    rows = [r for r in events if r["k"] == "s" and 44.9 <= r["t"] <= 92.1]
    t = np.array([r["t"] for r in rows])
    starts = [r["t"] for r in events if r.get("name") == "wheels"]
    stops = [r["t"] for r in events if r.get("why") == "segment_end"]
    ends = [0, *[np.searchsorted(t, v) - 1 for v in starts[1:]], len(t) - 1]
    gyro = np.degrees(
        np.loadtxt(DATA / "head-gyro.csv", delimiter=",", skiprows=1)[:, 1]
    )
    obs = relative_center(
        np.array([r["pose"][:2] for r in rows]), np.array([r["yaw"] for r in rows])
    )
    lo, hi = np.argmax(np.diff(t)), np.searchsorted(t, 91.49)
    t[lo : hi + 1] = np.linspace(t[lo], t[hi], hi - lo + 1)
    w = np.array([[r["wl"], r["wr"]] for r in rows])
    if mode == "balanced":
        h = (w[:, 1] - w[:, 0]) / 2
        w = np.column_stack((-h, h))
    clock, o = run(
        rows, t, w, 42, dense=True, include_pose=True, configure=conf, step_ms=step
    )
    xy = relative_center(o[:, 3:5], o[:, 5])
    at = np.column_stack([np.interp(t[ends], clock, u) for u in xy.T])
    dif = np.diff(at, axis=0)
    for i, (start, stop) in enumerate(zip(starts, stops)):
        pick = (t >= start + 1) & (t < stop - 0.1)
        error = np.interp(t[pick], clock, o[:, 0]) - gyro[pick]
        emit(i + 1, "drift_mm", np.linalg.norm(dif[i]))
        emit(
            i + 1,
            "vector_error_mm",
            np.linalg.norm(dif[i] - np.diff(obs[ends], axis=0)[i]),
        )
        emit(i + 1, "gyro_rmse_dps", np.mean(error**2) ** 0.5)
        emit(i + 1, "dx_mm", dif[i, 0])
        emit(i + 1, "dy_mm", dif[i, 1])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument(
        "--constant", action="store_true", help="Run smooth fixed-speed controls only"
    )
    parser.add_argument(
        "--interaction",
        action="store_true",
        help="Cross centered pressure patches with smaller solver steps",
    )
    parser.add_argument(
        "--refine", action="store_true", help="Check convergence at 0.125 ms"
    )
    parser.add_argument(
        "--native", action="store_true", help="Recompute the native checks alone"
    )
    args = parser.parse_args()
    args.directory.mkdir(exist_ok=True, parents=True)
    cases = [
        (shape, 1, 0, mode)
        for shape in ("stock", "narrow", "sphere", "paired", "sphere-plane")
        for mode in ("balanced", "measured", "native")
    ]
    cases += [
        (shape, 1, angle, "balanced")
        for shape in ("stock", "sphere", "sphere-plane")
        for angle in (30, 60)
    ]
    cases += [
        ("stock", step, 0, mode)
        for step in (0.5, 0.25)
        for mode in ("balanced", "measured", "native")
    ]
    if args.constant:
        cases = [
            (shape, 1, angle, "constant")
            for shape in ("stock", "narrow", "sphere", "paired", "sphere-plane")
            for angle in (0, 30, 60)
        ]
    if args.interaction:
        cases = [
            (shape, step, 0, mode)
            for shape in ("sphere", "paired")
            for step in (0.5, 0.25)
            for mode in ("balanced", "measured", "native")
        ]
    if args.refine:
        cases = [
            (shape, 0.125, 0, mode)
            for shape in ("stock", "sphere")
            for mode in ("balanced", "measured", "native")
        ]
    if args.native:
        cases = [case for case in cases if case[-1] == "native"]
    with ProcessPoolExecutor(4) as pool:
        for case, rows in zip(cases, pool.map(run_case, cases)):
            with (args.directory / ("-".join(map(str, case)) + ".csv")).open("w") as f:
                writer = csv.writer(f, lineterminator="\n")
                writer.writerow(
                    (
                        "geometry",
                        "step_ms",
                        "table_deg",
                        "input",
                        "turn",
                        "metric",
                        "value",
                    )
                )
                writer.writerows(rows)
            print(case, flush=True)


if __name__ == "__main__":
    main()

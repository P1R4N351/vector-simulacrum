"""Test tread-aligned sliding friction using the fixed B slope and measured wheels."""

import argparse
import csv
import ctypes
import json
import subprocess
import tempfile
from concurrent.futures import ProcessPoolExecutor
from functools import lru_cache
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

from plot_heading_crossover import BLOCKS, basis, optical
from replay_endurance_drift import relative_center
from replay_measured_turn import run
from replay_pivot import configure_round
from replay_shelf_tilt import OUT as TILT
from replay_shelf_tilt import estimate, read
from see3cam_shelf import DATA, camera_pose, edge_groups, lens_fit

OUT = DATA.parent / "lateral-friction"
# -1 uses the original implicit geom pairs and original solver tangents.
GRID = (-1, 0.65, 0.80, 0.85, 0.875, 0.8875, 0.90, 0.9125, 0.925, 0.95, 1.00, 1.15)
MODELS = ("Cylinder", "Sphere")


@lru_cache
def setup():
    lens = lens_fit(edge_groups())
    picks = np.loadtxt(
        DATA / "picks.csv", delimiter=",", skiprows=1, usecols=range(3, 7)
    )
    _, h, *_ = optical(picks, camera_pose(lens), lens)
    hs = dict(zip(BLOCKS, h.reshape(4, 3, 2)))
    return hs, estimate(("B0", "B180"), hs, read(TILT / "rest.csv"))


@lru_cache
def hook(path):
    lib = ctypes.CDLL(path)
    lib.install_tread_frame()
    return lib


def trial(task):
    turn, model, mu, lib, level = task
    hook(lib).enable_tread_frame(int(mu > 0))
    block, direction = turn["label"].split("_")
    heads, up = setup()
    if level:
        up = np.zeros(2)
    frame = basis(heads[block][int(direction == "cw")])

    def configure(root):
        if model == "Sphere":
            configure_round(root)
        opt = root.find("option")
        g = np.linalg.norm(np.fromstring(opt.get("gravity"), sep=" "))
        opt.set(
            "gravity", " ".join(map(str, -g * np.r_[frame @ up, np.sqrt(1 - up @ up)]))
        )
        if mu < 0:
            return
        contacts = root.find("contact")
        if contacts is None:
            contacts = ET.SubElement(root, "contact")
        for patch in root.findall(".//geom"):
            name = patch.get("name", "")
            if not name.startswith("wheel_patch_"):
                continue
            longitudinal, spin, roll = patch.get("friction").split()
            assert float(longitudinal) == 0.9
            ET.SubElement(
                contacts,
                "pair",
                geom1="table",
                geom2=name,
                condim="3",
                solref=patch.get("solref"),
                friction=f"{longitudinal} {mu} {spin} {roll} {roll}",
            )

    obs = turn["rows"]
    t = np.array([r["t"] for r in obs])
    w = np.array([[r["wl"], r["wr"]] for r in obs])
    count = 0
    verified = 0

    def audit(m, d, now):
        nonlocal count, verified
        count += 1
        if count % 127:
            return
        x = d.xmat[m.body("chassis").id].reshape(3, 3)[:, 0]
        for c in d.contact:
            if not any(
                (m.geom(i).name or "").startswith("wheel_patch_") for i in c.geom
            ):
                continue
            n, u, _ = c.frame.reshape(3, 3)
            expected = x - n * (x @ n)
            if abs(u @ expected) / np.linalg.norm(expected) < 0.9999 or not np.allclose(
                c.friction[:2], [0.9, mu]
            ):
                raise ValueError(
                    f"Tread-frame audit failed: {model}, {mu}, {u}, {expected}, {c.friction}"
                )
            verified += 1

    clock, out = run(
        obs,
        t,
        w,
        42,
        dense=True,
        include_pose=True,
        configure=configure,
        step_ms=1 if model == "Cylinder" else 0.125,
        observe=audit if mu > 0 else None,
    )
    if mu > 0:
        assert verified > 0
    xy = relative_center(out[:, 3:5], out[:, 5])[-1] @ frame
    pred = np.interp(t, clock, out[:, 0])
    gyro = np.degrees([r["gyro"][2] for r in obs])
    steady = (t > turn["start"] + 0.5) & (t < turn["stop"] - 0.2)
    return {
        "block": block,
        "spin": direction.upper(),
        "model": model,
        "mu": mu,
        "x": xy[0],
        "y": xy[1],
        "gyro_mse": np.mean((pred - gyro) ** 2),
        "steady_bias": np.mean(pred[steady] - gyro[steady]),
        "time": (t - turn["start"]).tolist(),
        "pred": pred.tolist(),
        "gyro": gyro.tolist(),
    }


def metrics(rows):
    cam = read(TILT / "vectors.csv")
    errs = []
    per_turn = []
    for block in sorted({r["block"] for r in rows}):
        group = [r for r in rows if r["block"] == block]
        deltas = []
        for r in group:
            ref = next(
                s
                for s in cam
                if (s["block"], s["spin"], s["model"]) == (block, r["spin"], r["model"])
            )
            deltas.append(
                np.array([r["x"], r["y"]])
                - np.array([float(ref["camera_x"]), float(ref["camera_y"])])
            )
        errs.append(np.linalg.norm(np.sum(deltas, axis=0)))
        per_turn.extend(np.linalg.norm(deltas, axis=1))
    return [
        np.sqrt(np.mean(np.square(errs))),
        np.sqrt(np.mean(np.square(per_turn))),
        np.sqrt(np.mean([r["gyro_mse"] for r in rows])),
        np.mean([abs(r["steady_bias"]) for r in rows]),
    ]


def save_csv(name, fields, rows):
    with (OUT / name).open("w") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(fields)
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--level", action="store_true")
    level = parser.parse_args().level
    prefix = "level-" if level else ""
    root = Path(mujoco.__file__).parent
    dylib = next(
        p
        for p in root.iterdir()
        if p.name.startswith("libmujoco.") and ("dylib" in p.name or ".so" in p.name)
    )
    turns = json.loads((DATA / "turns.json").read_text())
    with tempfile.TemporaryDirectory() as tmp:
        lib = str(Path(tmp) / "tangent.so")
        subprocess.run(
            [
                "cc",
                "-shared",
                "-O2",
                "-fPIC",
                "-I" + str(root / "include"),
                str(OUT / "tangent.c"),
                str(dylib),
                "-Wl,-rpath," + str(root),
                "-o",
                lib,
            ],
            check=True,
        )
        with ProcessPoolExecutor(max_workers=6) as pool:
            b = list(
                pool.map(
                    trial,
                    [
                        (t, m, mu, lib, level)
                        for mu in GRID
                        for m in MODELS
                        for t in turns
                        if t["label"].startswith("B")
                    ],
                )
            )
            summary = []
            chosen = {}
            for m in MODELS:
                scores = {
                    mu: metrics([r for r in b if r["model"] == m and r["mu"] == mu])
                    for mu in GRID
                }
                for mu, s in scores.items():
                    summary.append(["B", m, mu, *s])
                # Compare to both the legacy baseline and the aligned isotropic control.
                good = [
                    mu
                    for mu in GRID
                    if mu > 0
                    and mu != 0.9
                    and all(
                        scores[mu][0] < scores[base][0]
                        and all(scores[mu][i] <= scores[base][i] + 1 for i in (2, 3))
                        for base in (-1, 0.9)
                    )
                ]
                choices = good or [mu for mu in GRID if mu > 0 and mu != 0.9]
                chosen[m] = min(choices, key=lambda mu: scores[mu][0])
                print(
                    m,
                    "B",
                    scores,
                    "transfer",
                    chosen[m],
                    "passed B gate",
                    bool(good),
                    flush=True,
                )
            a = list(
                pool.map(
                    trial,
                    [
                        (t, m, mu, lib, level)
                        for m in MODELS
                        for mu in (-1, 0.9, chosen[m])
                        for t in turns
                        if t["label"].startswith("A")
                    ],
                )
            )
    for m in MODELS:
        for mu in (-1, 0.9, chosen[m]):
            summary.append(
                [
                    "A",
                    m,
                    mu,
                    *metrics([r for r in a if r["model"] == m and r["mu"] == mu]),
                ]
            )
    save_csv(
        prefix + "scores.csv",
        [
            "spot",
            "model",
            "lateral_mu",
            "pair_rmse_mm",
            "turn_rmse_mm",
            "gyro_rmse_dps",
            "steady_abs_bias_dps",
        ],
        summary,
    )
    save_csv(
        prefix + "endpoints.csv",
        ["block", "spin", "model", "lateral_mu", "right_mm", "dockward_mm"],
        [[r[k] for k in ("block", "spin", "model", "mu", "x", "y")] for r in b + a],
    )
    save_csv(
        prefix + "gyro.csv",
        [
            "block",
            "spin",
            "model",
            "lateral_mu",
            "seconds",
            "observed_dps",
            "predicted_dps",
        ],
        (
            [r["block"], r["spin"], r["model"], r["mu"], t, g, p]
            for r in a
            for t, g, p in zip(r["time"], r["gyro"], r["pred"])
        ),
    )
    print(summary[-6:])


if __name__ == "__main__":
    main()

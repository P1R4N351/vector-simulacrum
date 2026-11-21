"""Audit the effect of head posture on the stationary and turning IMU data."""

import json

import numpy as np

from replay_lateral_friction import BLOCKS, DATA, OUT, TILT, basis, read, setup

hs, up = setup()


def xy(rows, zero=False):
    a = np.array([[float(r[k]) for k in ("ax", "ay", "az")] for r in rows])
    h = np.array([float(r["head_rad"]) for r in rows]) * (not zero)
    return (
        np.mean(np.c_[a[:, 0] * np.cos(h) - a[:, 2] * np.sin(h), a[:, 1]], axis=0)
        / 9810
    )


pre = read(TILT / "rest.csv")
rest = read(OUT / "after-stop-rests.csv")
print(
    "post rejected",
    sum(
        abs(float(r["wl"])) + abs(float(r["wr"])) > 0
        or np.linalg.norm([float(r[k]) for k in ("gx", "gy", "gz")]) > 0.05
        for r in rest
    ),
)
for spot in "BA":
    blocks = [spot + "0", spot + "180"]
    groups = [[r for r in pre if r["block"] == b] for b in blocks]
    frames = [basis(hs[b][0]) for b in blocks]
    for zero in (True, False):
        a, b = [xy(g, zero) for g in groups]
        u = np.linalg.solve(frames[0] - frames[1], a - b)
        print(
            spot,
            "ignore head" if zero else "correct",
            np.degrees(np.arcsin(np.linalg.norm(u))),
            u * 100,
        )
fixed = np.mean(
    [
        xy([r for r in pre if r["block"] == b]) - basis(hs[b][0]) @ up
        for b in ["B0", "B180"]
    ],
    axis=0,
)
level = np.mean(
    [xy([r for r in pre if r["block"] == b]) for b in ["B0", "B180"]], axis=0
)
for spot in "BA":
    err = []
    for b in BLOCKS:
        if not b.startswith(spot):
            continue
        for j in (1, 2):
            rows = [
                r
                for r in rest
                if r["block"] == b
                and int(r["rest"]) == j
                and float(r["wl"]) == float(r["wr"]) == 0
                and np.linalg.norm([float(r[k]) for k in ("gx", "gy", "gz")]) < 0.05
            ]
            obs = xy(rows)
            err.append(
                [
                    np.linalg.norm(obs - level),
                    np.linalg.norm(obs - fixed - basis(hs[b][j]) @ up),
                ]
            )
    print(
        spot,
        "post-stop level/effective RMS mg",
        np.sqrt(np.mean(np.square(err), axis=0)) * 1000,
    )
for turn in json.loads((DATA / "turns.json").read_text()):
    a = np.array([r["head"] for r in turn["rows"]])
    print(
        turn["label"],
        "head range",
        np.degrees([min(a), max(a)]),
        "max z/cos(head)-z (dps)",
        max(
            abs(np.degrees(r["gyro"][2]) * (1 / np.cos(r["head"]) - 1))
            for r in turn["rows"]
        ),
    )

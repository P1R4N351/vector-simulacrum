"""Replay image-plane rotations on the released native-pixel side-insert contours."""

import csv
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree


def fit(reference, observed, sign):
    a, b = reference, observed

    # Partial overlap: compare the common upper arc in both directions, letting endpoints vary.
    def trans(p):
        th = np.radians(p[0])
        return np.array([[np.cos(th), np.sin(th)], [-np.sin(th), np.cos(th)]])

    def res(p):
        at = a @ trans(p) + p[1:]
        d = cKDTree(b).query(at)[0]
        e = cKDTree(at).query(b)[0]
        return np.r_[np.sort(d)[: int(0.88 * len(d))], np.sort(e)[: int(0.88 * len(e))]]

    p = min(
        (
            least_squares(
                res,
                [k, *(b.mean(0) - a.mean(0) @ trans([k]))],
                diff_step=0.001,
                max_nfev=160,
            )
            for k in np.arange(-60, 61, 15)
        ),
        key=lambda v: v.cost,
    )
    return p.x[0] * sign, np.sqrt(np.mean(res(p.x) ** 2))


if __name__ == "__main__":
    with (
        Path(__file__).parent / "evidence/face-tag-trial/phone-edges.csv"
    ).open() as f:
        rows = list(csv.DictReader(f))
    for view in ("Brev", "Aside", "Arev"):
        frames = {}
        for row in rows:
            if row["view"] == view:
                frames.setdefault(row["frame"], []).append(
                    [float(row["u"]), float(row["v"])]
                )
        baseline = next(name for name in frames if "-0-" in name)
        for name, points in frames.items():
            if name != baseline:
                angle, rms = fit(
                    np.array(frames[baseline]),
                    np.array(points),
                    -1 if view == "Aside" else 1,
                )
                print(view, name, round(angle, 2), round(rms, 2))

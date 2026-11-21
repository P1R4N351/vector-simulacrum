"""Sensitivity of rigid pitch to an assumed local backpack surface profile."""

import csv
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from see3cam_shelf import camera_pose, edge_groups, image_points, lens_fit, locate

root = Path(__file__).resolve().parent / "evidence"


def read(f):
    with f.open() as fd:
        return list(csv.DictReader(fd))


tracks = read(root / "face-tag-trial/track-pixels.csv")
marks = {r["block"]: r for r in read(root / "head-sweep/marks.csv")}
lens = lens_fit(edge_groups())
pose = camera_pose(lens)
for slope in [-0.5, 0, 0.5]:
    vals = {}
    for block in sorted({r["block"] for r in tracks}):
        mark = marks[block]
        xy = [
            locate([[float(mark[k + "_u"]), float(mark[k + "_v"])]], z, pose, lens)[0]
            for k, z in [("strip", 65.5), ("vent", 64.5)]
        ]
        h = xy[1] - xy[0]
        h /= np.linalg.norm(h)
        angles = []
        for visit in [3, 4]:
            group = [
                r for r in tracks if r["block"] == block and int(r["visit"]) == visit
            ]
            uv, flow = [
                np.array([[float(r[k]) for k in pair] for r in group])
                for pair in [("initial_u", "initial_v"), ("du_px", "dv_px")]
            ]
            flat = locate(uv, 65.5, pose, lens)
            z = 65.5 + np.clip(slope * ((flat - xy[0]) @ h), -7, 7)
            xyz = np.array(
                [
                    [*locate([px], height, pose, lens)[0], height]
                    for px, height in zip(uv, z)
                ]
            )
            mid = xyz.mean(0)
            base = image_points(xyz, pose, lens)
            axes = [np.r_[-h[1], h[0], 0], np.r_[h, 0], np.array([0, 0, 1])]
            d = (
                np.stack(
                    [
                        image_points(
                            Rotation.from_rotvec(a * np.pi / 180 * 0.01).apply(
                                xyz - mid
                            )
                            + mid,
                            pose,
                            lens,
                        )
                        - base
                        for a in axes
                    ]
                    + [
                        image_points(xyz + a * 0.01, pose, lens) - base
                        for a in np.eye(3)
                    ],
                    axis=-1,
                )
                / 0.01
            )
            angles.append(
                -np.linalg.lstsq(d.reshape(-1, 6), flow.ravel(), rcond=None)[0][0]
            )
        vals[block] = angles[1] - angles[0]
    print(
        slope,
        {
            s: round(float(np.median([v for b, v in vals.items() if b[0] == s])), 3)
            for s in "BA"
        },
    )

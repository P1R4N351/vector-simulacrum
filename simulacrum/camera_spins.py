"""Where unit 3 turns about in its four camera-recorded spins. Authored-by: CPCS.

    python3 -m simulacrum.camera_spins

Needs numpy and scipy, not MuJoCo.

The footage and the picks are gptpcs-01a1f1b's (evidence/camera-spins in
that lane's tree, commit 33ff9ea; copied to
evidence/camera-spins-unit3-20261120). A fixed camera looked across the
shelf at unit 3 while it spun four times at 150 mm/s; that lane picked the
forward end of the backpack lens in 35 keyframes, and a second point along
the roof (the aim point) for heading. The camera's model and the dock
references below are copied unchanged from that lane's
replay_camera_spins.py: a local equidistant fisheye with the principal
point at the image centre, posed from the dock glyph's four corners and the
ramp's front centre, with two shelf-edge constraints.

Each lens pick is intersected with the plane 65.5 mm above the shelf (that
lane's lens height). Within one spin the lens moves on a circle about the
turn centre, so a circle fitted to its path gives the centre without using
any heading; the aim point then turns the centre into the body frame. The
lens is LENS_X_MM behind the front axle, which rests on that lane's
geometric model of unit 3.
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent.parent
PICKS = ROOT / "evidence" / "camera-spins-unit3-20261120" / "camera-picks.csv"
LENS_X_MM = -36.3
LENS_HEIGHT_MM = 65.5
# World X right, Y away from the camera, Z up, origin on the shelf under the dock glyph (mm).
WORLD = np.array([[-23, 0, 71.5], [23, 0, 71.5], [23, 0, 25.5], [-23, 0, 25.5], [0, -94, 0]], float)
GLYPH_LIP = np.array([[829, 542], [902, 543], [901, 611], [831, 611], [862, 717]], float)
SHELF = np.array([[622, 610], [1116, 622], [456, 720], [1274, 746]], float)
PRINCIPAL = np.array([960.0, 540.0])
START_POSE = [1.79, -0.08, 0.12, -62, 71, 389, 595]


def project(xyz: np.ndarray, pose: np.ndarray) -> np.ndarray:
    """World points (mm) to pixels."""
    assert xyz.ndim == 2 and xyz.shape[1] == 3, "points are rows of x, y, z"
    ray = Rotation.from_rotvec(pose[:3]).apply(xyz) + pose[3:6]
    xy = ray[:, :2] / ray[:, 2, None]
    h = np.linalg.norm(xy, axis=1)[:, None]
    return PRINCIPAL + pose[6] * xy * np.arctan(h) / np.maximum(h, 1e-9)


def intersect(uv: np.ndarray, z: float, pose: np.ndarray) -> np.ndarray:
    """Pixels to the x, y (mm) where their rays meet the plane at height z."""
    assert uv.ndim == 2 and uv.shape[1] == 2, "pixels are rows of u, v"
    xy = (uv - PRINCIPAL) / pose[6]
    h = np.linalg.norm(xy, axis=1)[:, None]
    ray = np.column_stack((xy * np.tan(h) / np.maximum(h, 1e-9), np.ones(len(h))))
    inv = Rotation.from_rotvec(pose[:3]).inv()
    centre, direction = inv.apply(-pose[3:6]), inv.apply(ray)
    return (centre + direction * ((z - centre[2]) / direction[:, 2])[:, None])[:, :2]


def calibration(focal_prior: float = 580.0, edge_weight: float = 4.0) -> np.ndarray:
    """Camera pose and focal length from the dock references and the shelf edges."""
    assert focal_prior > 0.0 and edge_weight > 0.0, "prior and weight must be positive"

    def residual(p: np.ndarray) -> np.ndarray:
        a, b, c, d = intersect(SHELF, 0.0, p)
        across = (b - a) / np.linalg.norm(b - a)
        return np.r_[((project(WORLD, p) - GLYPH_LIP) / 1.5).ravel(), (p[-1] - focal_prior) / 130,
                     ((c - a) @ across) / edge_weight, ((d - b) @ across) / edge_weight]

    fit = least_squares(residual, START_POSE, bounds=([-7] * 3 + [-2000] * 2 + [50, 300], [7] * 3 + [2000] * 3 + [1300]))
    assert fit.success, "the camera calibration did not converge"
    return fit.x


def circle(points: np.ndarray) -> np.ndarray:
    """Least-squares circle through points (Kasa, then geometric): centre x, centre y, radius."""
    assert points.ndim == 2 and len(points) >= 3, "a circle needs three points"
    x, y = points[:, 0], points[:, 1]
    a = np.column_stack((2.0 * x, 2.0 * y, np.ones(len(x))))
    cx, cy, c = np.linalg.lstsq(a, x * x + y * y, rcond=None)[0]
    fit = least_squares(lambda q: np.hypot(x - q[0], y - q[1]) - q[2], [cx, cy, np.sqrt(c + cx * cx + cy * cy)])
    return fit.x


def turn_centres(picks: Path = PICKS, focal_prior: float = 580.0, height: float = LENS_HEIGHT_MM) -> Dict[str, Any]:
    """Per spin: the turn centre's x in the body frame, its sideways offset, and the lens circle's radius."""
    with open(picks, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert rows and {"turn", "u_px", "v_px", "aim_u_px", "aim_v_px"} <= set(rows[0]), "the picks file lacks a column"
    pose = calibration(focal_prior)
    lens = intersect(np.array([[float(r["u_px"]), float(r["v_px"])] for r in rows]), height, pose)
    ahead = intersect(np.array([[float(r["aim_u_px"]), float(r["aim_v_px"])] for r in rows]), height, pose) - lens
    ahead /= np.linalg.norm(ahead, axis=1)[:, None]
    spins: List[Dict[str, float]] = []
    for turn in sorted({r["turn"] for r in rows}, key=int):
        idx = [i for i, r in enumerate(rows) if r["turn"] == turn]
        cx, cy, radius = circle(lens[idx])
        rel = np.array([cx, cy]) - lens[idx]
        fwd = np.sum(rel * ahead[idx], axis=1)
        left = ahead[idx][:, 0] * rel[:, 1] - ahead[idx][:, 1] * rel[:, 0]
        spins.append({"turn": int(turn), "keyframes": len(idx), "radius_mm": float(radius),
                      "centre_x_mm": LENS_X_MM + float(np.median(fwd)), "centre_left_mm": float(np.median(left))})
    assert len(spins) == 4, "the recording has four spins"
    return {"focal_px": float(pose[6]), "spins": spins}


if __name__ == "__main__":
    print(json.dumps(turn_centres(), indent=1, sort_keys=True))
    sys.exit(0)

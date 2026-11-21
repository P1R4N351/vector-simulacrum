"""Reproduce the A-positive brow check from the published pixel and IMU observations."""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def fit_chords(points, low_deg):
    """Predict a third point from a known first rotation about the lateral axis."""
    first, second = np.diff(points, axis=1).transpose(1, 0, 2)
    candidates = np.arange(0.0, 75.1, 0.1)
    lo = np.radians(low_deg)[:, None] / 2
    hi = np.radians(candidates)[None, :] / 2
    ratio = np.sin(hi) / np.sin(lo)
    cosine, sine = ratio * np.cos(lo + hi), ratio * np.sin(lo + hi)
    x, z = first.T[:, :, None]
    predicted = np.stack((x*cosine - z*sine, x*sine + z*cosine), axis=-1)
    errors = np.linalg.norm(predicted - second[:, None], axis=-1)
    return candidates[np.argmin(errors, axis=1)], errors[:, [270, 340]]


def estimate(data, uv, low, yaw):
    origin, directions = [], []
    forward = Rotation.from_euler("z", yaw[:, None], degrees=True).apply(np.broadcast_to(data["forward"], (len(yaw), 3)))
    normal = np.cross([0, 0, 1], forward)
    estimates = []
    for i, camera in enumerate(data["cameras"]):
        inv = Rotation.from_rotvec(camera["rotation"]).inv()
        origin.append(inv.apply(-np.array(camera["translation"])))
        xy = (uv[:, :, i] * data["native_to_working"][i] - camera["principal_point"]) / camera["focal_px"]
        d = inv.apply(np.dstack((xy, np.ones_like(xy[:, :, 0]))).reshape(-1, 3)).reshape(*xy.shape[:2], 3)
        d /= np.linalg.norm(d, axis=-1, keepdims=True)
        directions.append(d)
        # Intersect each ray with a parallel sagittal plane at arbitrary distance.
        # Changing the distance applies a uniform scale and translation to the trajectory.
        p = d / np.einsum("npc,nc->np", d, normal)[:, :, None]
        xz = np.stack((np.einsum("npc,nc->np", p, forward), p[:, :, 2]), axis=-1)
        estimates.append(fit_chords(xz, low))

    matrices = [np.eye(3) - d[..., :, None] * d[..., None, :] for d in directions]
    rhs = sum(m @ o for m, o in zip(matrices, origin))
    world = np.linalg.solve(sum(matrices), rhs[..., None])[..., 0]
    xz = np.stack((np.einsum("npc,nc->np", world, forward), world[:, :, 2]), axis=-1)
    estimates.append(fit_chords(xz, low))
    return estimates


def run(data):
    shifts = np.array([[np.mean([r["delta"] for r in batch], axis=0)
                        for batch in data["registration"]["Ap"][g]]
                       for g in data["camera_order"]]).transpose(1, 0, 2)
    pixels = np.array(data["picks"]) - shifts
    rest = data["stationary"]["Ap"]
    low = rest[1]["pitch_deg"] - rest[0]["pitch_deg"]
    best = estimate(data, pixels[None], np.array([low]), np.zeros(1))
    rng = np.random.default_rng(19)
    n = 2000
    # Pick and pose sensitivity assumptions, not empirically calibrated confidence limits.
    uv = pixels + rng.normal(size=(n, 3, 2, 2)) * np.array([2., 1.])[:, None]
    draws = estimate(data, uv, rng.normal(low, .5, n), rng.normal(0., 3., n))
    for name, (angle, errors), (spread, _) in zip(data["camera_order"] + ["stereo"], best, draws):
        print(name, "deg", angle[0], "sensitivity P05/P95", np.round(np.percentile(spread, [5, 95]), 1))
        if name == "stereo":
            print("conditional sagittal endpoint errors, 27 and 34 deg (mm)", np.round(errors[0], 2))


if __name__ == "__main__":
    run(json.loads(Path(__file__).with_name("observations.json").read_text()))

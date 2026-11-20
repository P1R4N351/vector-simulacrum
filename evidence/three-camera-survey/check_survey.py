"""Recompute the landmark triangulations from survey.json (NumPy and SciPy)."""

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def triangulate(rays):
    projectors = [np.eye(3) - np.outer(d, d) for _, d in rays]
    return np.linalg.solve(sum(projectors), sum(m @ o for m, (o, _) in zip(projectors, rays)))


def ray(observation, camera):
    inv = Rotation.from_rotvec(camera["rotation"]).inv()
    origin = inv.apply(-np.array(camera["translation"]))
    direction = inv.apply(np.r_[(np.array(observation) - camera["principal_point"]) / camera["focal_px"], 1.0])
    return origin, direction / np.linalg.norm(direction)


def project(point, camera):
    v = Rotation.from_rotvec(camera["rotation"]).apply(point) + camera["translation"]
    return v[:2] / v[2] * camera["focal_px"] + camera["principal_point"]


def pitch(degrees):
    return Rotation.from_euler("y", -degrees, degrees=True)


def check(data):
    cameras = data["cameras"]
    for label, observations in data["microphone_ports"].items():
        points = []
        for j in range(2):
            used = [(camera, np.array(uv[j])) for camera, uv in zip(cameras, observations) if uv]
            rays = [ray(uv, camera) for camera, uv in used]
            point = triangulate(rays)
            points.append(point)
            if len(rays) == 3:
                errors = [np.linalg.norm(project(triangulate(rays[:i] + rays[i+1:]), c) - uv)
                          for i, (c, uv) in enumerate(used)]
                print(label, j, "omitted-camera errors (px)", np.round(errors, 1))
        print(label, "microphone separation (mm, conditional on calibration)",
              round(np.linalg.norm(points[1] - points[0]), 2))

    # Express each viewing ray in the neutral head frame. In the photo the
    # far bank changes sides when the robot faces the opposite direction.
    hinge = np.array([25.3, 0.0, -17.5])  # relative to indicator tip, mm
    neutral = []
    for j in range(4):
        rays = []
        for v in data["crown"].values():
            if v["split"] != "fit":
                continue
            i = j if v["far_bank_sign"] > 0 else (j + 2) % 4
            o, d = ray(v["end_slots_px"][i], cameras[2])
            axes = np.asarray(v["axes"])
            inv = pitch(v["encoder_deg"]).inv()
            rays.append((inv.apply(axes @ (o - v["fixed_origin"]) - hinge) + hinge,
                         inv.apply(axes @ d)))
        neutral.append(triangulate(rays))
    print("crown endpoints relative to fixed strip tip (mm)", np.round(neutral, 2))
    for name, v in data["crown"].items():
        points = pitch(v["encoder_deg"]).apply(np.array(neutral) - hinge) + hinge
        world = points @ v["axes"] + v["fixed_origin"]
        pixels = np.array([project(p, cameras[2]) for p in world])
        if v["far_bank_sign"] < 0:
            pixels = np.roll(pixels, 2, axis=0)
        residual = pixels - v["end_slots_px"]
        rms = np.sqrt(np.mean(np.sum(residual**2, axis=1)))
        print(name, v["split"], "free triangulation reprojection RMS (px)", round(rms, 2))


if __name__ == "__main__":
    check(json.loads(Path(__file__).with_name("survey.json").read_text()))

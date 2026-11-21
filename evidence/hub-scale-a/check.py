"""Recompute the scale checks and held-out microphone chords from published picks."""
import json
import sys
from pathlib import Path

import numpy as np

here = Path(__file__).resolve().parent
survey = here.parent / "three-camera-survey"
sys.path.insert(0, str(survey))
from check_survey import ray, triangulate

data = json.loads((here / "measurements.json").read_text())
original = json.loads((survey / "survey.json").read_text())
unit = np.array([[0, 0], [46, 0], [46, 46], [0, 46]])
widths = []
for lens in ("main", "ultra"):
    entry = data[lens]
    equations = []
    values = []
    for (x, y), (u, v) in zip(entry["glyph"], unit):
        equations.extend(([x, y, 1, 0, 0, 0, -u*x, -u*y],
                          [0, 0, 0, x, y, 1, -v*x, -v*y]))
        values.extend((u, v))
    H = np.append(np.linalg.solve(equations, values), 1).reshape(3, 3)
    endpoints = np.column_stack((entry["wall_endpoints"], np.ones(2))) @ H.T
    endpoints = endpoints[:, :2] / endpoints[:, 2, None]
    widths.append(np.linalg.norm(endpoints[1] - endpoints[0]))
    print(lens, "upright on glyph scale (mm)", round(widths[-1], 2))
factor = np.mean(70 / np.array(widths))
for key in ("ruler1", "ruler2"):
    picks = data[key]
    ticks = np.array(picks["ticks20_80_crop_px"])
    hubs = np.array(picks["hubs_crop_px"])
    print(key, "apparent longitudinal span (mm)", round(60 * abs(hubs[1, 0] - hubs[0, 0]) / np.linalg.norm(ticks[1] - ticks[0]), 2))
cameras = original["cameras"]
for i, params in enumerate(data["no_hub_camera_params"]):
    cameras[i].update(rotation=params[:3], translation=params[3:6], focal_px=params[6])
for label, observations in original["microphone_ports"].items():
    ports = [triangulate([ray(uv[k], cam) for cam, uv in zip(cameras, observations) if uv])
             for k in range(2)]
    spacing = np.linalg.norm(ports[1] - ports[0])
    print(label, "glyph / upright (mm)", round(spacing, 2), round(spacing * factor, 2))

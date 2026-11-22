"""Register the backpack feature from opposing views using the B rim calibration."""
import csv
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parent / "three-camera-survey"))
from check_survey import project, ray

# Rectified See3CAM crop coordinates from the original crown survey (tip, front, rear).
views = {
    "Bp0": ([591, 324], [705, 555], [532, 550], 1),
    "Bm0": ([610, 319], [470, 509], [628, 552], -1),
    "ring165": ([645, 343], [495, 523], [652, 597], -1),
    "ring180": ([648, 335], [509, 553], [685, 582], -1),
    "ring345": ([567, 311], [633, 568], [485, 506], 1),
    "Am0": ([436, 297], [284, 466], [423, 507], -1),
}


def infer(camera):
    def at_height(pixel, height):
        origin, direction = ray(pixel, camera)
        return origin + direction * ((height - origin[2]) / direction[2])

    poses = {}
    for name, (tip, front, rear, side) in views.items():
        axle = at_height(front, 14.5)
        axis = axle - at_height(rear, 10.75)
        forward = axis[:2] / np.linalg.norm(axis[:2])
        lateral = side * np.array([-forward[1], forward[0]])
        poses[name] = axle, forward, lateral

    def residual(name, params):
        axle, forward, lateral = poses[name]
        x, y, height = params
        point = np.r_[axle[:2] + x * forward + y * lateral, height]
        return project(point, camera) - views[name][0]

    training = [name for name in views if name != "Am0"]
    fit = least_squares(lambda p: np.concatenate([residual(n, p) for n in training]),
                        [-36.3, -32, 65.5])
    return fit.x, {name: np.linalg.norm(residual(name, fit.x)) for name in views}


rows = []
for variant in ("cameras", "face-land", "omit-ring180"):
    camera = json.loads((here.parent / "hub-calibration-b" / (variant + ".json")).read_text())["cameras"][2]
    (x, lateral, height), errors = infer(camera)
    rows.append([variant, x, lateral, height, np.sqrt(np.mean(np.square(list(errors.values())[:-1]))), errors["Am0"]])
with (here / "landmark-registration.csv").open("w", newline="") as handle:
    writer = csv.writer(handle)
    writer.writerow(["rig", "tip_x_mm", "lateral_from_rim_mm", "height_mm", "B_rms_px", "A_error_px"])
    writer.writerows(rows)
print(rows)

# Independent Am0 read on the native 4000 px-wide Flip ultrawide still.
from check_survey import triangulate
ultra = np.array([[1562, 1456], [1274, 1814], [1355, 1911]]) / 4
see = np.array(views['Am0'][:3])
with (here / 'landmark-stereo.csv').open('w', newline='') as handle:
    writer = csv.writer(handle)
    writer.writerow(['rig', 'tip_x_mm', 'height_mm', 'hub_span_mm', 'reprojection_rms_px'])
    for variant in ('cameras', 'face-land', 'omit-ring180'):
        cameras = json.loads((here.parent / 'hub-calibration-b' / (variant + '.json')).read_text())['cameras'][1:]
        points = [triangulate([ray(u, cameras[0]), ray(s, cameras[1])]) for u, s in zip(ultra, see)]
        tip, front, rear = points
        axis = (front - rear)[:2]
        span = np.linalg.norm(axis)
        errors = [np.linalg.norm(project(point, camera) - uv)
                  for camera, pixels in zip(cameras, (ultra, see)) for point, uv in zip(points, pixels)]
        writer.writerow([variant, (tip-front)[:2] @ axis / span, tip[2], span, np.sqrt(np.mean(np.square(errors)))])

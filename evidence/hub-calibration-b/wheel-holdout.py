"""Triangulate free hub centers at A without imposing a wheelbase or axle height."""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SURVEY = HERE.parent / "three-camera-survey"
sys.path.insert(0, str(SURVEY))
from check_survey import project, ray, triangulate

def check(cameras, observations):
    output = {}
    for frame, rows in observations.items():
        available = [(camera, pair) for camera, pair in zip(cameras, rows) if pair]
        hubs = [triangulate([ray(pair[i], cam) for cam, pair in available]) for i in (0, 1)]
        delta = hubs[1] - hubs[0]
        errors = [[np.linalg.norm(project(point, cam) - xy) for point, xy in zip(hubs, pair)]
                  for cam, pair in available]
        output[frame] = {"horizontal_mm": float(np.linalg.norm(delta[:2])),
                         "center_to_center_mm": float(np.linalg.norm(delta)),
                         "rear_minus_front_height_mm": float(delta[2]),
                         "camera_rms_px": np.sqrt(np.mean(np.square(errors), axis=1)).tolist()}
    return output

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibration", type=Path)
    args = parser.parse_args()
    config = json.loads(args.calibration.read_text())
    cameras = config["cameras"] if isinstance(config, dict) else config
    picks = json.loads((HERE / "wheel-picks.json").read_text())
    print(json.dumps(check(cameras, picks), indent=2))

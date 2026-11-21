"""Compare exported SDK-frame rear shell vertices on the same recorded camera rays."""
import json
import sys
from pathlib import Path
from zipfile import ZipFile

import cv2
import numpy as np
from scipy.interpolate import LinearNDInterpolator
from scipy.ndimage import map_coordinates


def read_mesh(path):
    # The SDK OBJ front axle is 2.947435 mm forward of file origin.
    points = np.array(json.loads(Path(path).read_text())).reshape(141, 85, 3)
    return points * 10 - [2.947435, 0, 0]


def project(points, camera):
    optical = points @ np.array(camera['R']).T + camera['t']
    xy = optical[..., :2] / optical[..., 2, None] * camera['focal'] + camera['principal_in_crop']
    return xy, optical[..., 2]


def shoulder_minus_center(values):
    sides = (np.median(values[:, :7], axis=1) + np.median(values[:, 8:], axis=1)) / 2
    return sides - values[:, 7]


def predicted_depth(mesh, camera, pixels):
    xy, depth = project(mesh[34:100, 3:-3].reshape(-1, 3), camera)
    return LinearNDInterpolator(xy, depth)(pixels)


if __name__ == '__main__':
    directory = Path(__file__).parent
    cameras = json.loads((directory / 'profile-cameras.json').read_text())
    before, after = map(read_mesh, sys.argv[1:3])
    columns = [*range(10, 17), 42, *range(68, 75)]
    with ZipFile(directory / 'scan.zip') as archive:
        for label, camera in cameras.items():
            image = np.frombuffer(archive.read(label + '/depth.png'), dtype=np.uint8)
            z = cv2.imdecode(image, cv2.IMREAD_UNCHANGED).astype(float) / 10
            z[z == 0] = np.nan
            xy, _ = project(before[48:88, columns], camera)
            predicted = [shoulder_minus_center(predicted_depth(m, camera, xy)) for m in (before, after)]
            coords = (xy + [0, 5]).transpose(2, 0, 1)[::-1]
            observed = shoulder_minus_center(map_coordinates(z, coords, order=1))
            print(label, *(round(np.mean(np.abs(observed - p)), 2) for p in predicted))

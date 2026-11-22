"""Measure the strip tip on the independent three-camera repeat at shelf spot A."""
import csv
from itertools import combinations
import json
from pathlib import Path
import sys

import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parent / 'three-camera-survey'))
from check_survey import project, ray, triangulate

rig = here.parent / 'hub-calibration-b'
hubs = json.loads((rig / 'wheel-picks.json').read_text())['S-Am-15']
# Native main, native ultrawide, and rectified See3CAM coordinates.
tips = np.array([[2805, 2586], [1613, 1447], [482.5, 299.5]]) / np.array([[8.16], [4], [1]])
pixels = np.concatenate([tips[:, None, :], hubs], axis=1)
names = ['main', 'ultra', 'see']
with (here / 'repeat-landmark.csv').open('w', newline='') as handle:
    w = csv.writer(handle)
    w.writerow(['rig', 'views', 'tip_x_mm', 'tip_height_mm', 'hub_span_mm', 'omitted_tip_error_px'])
    for variant in ('cameras', 'face-land', 'omit-ring180'):
        cameras = json.loads((rig / (variant + '.json')).read_text())['cameras']
        for group in [tuple(range(3)), *combinations(range(3), 2)]:
            tip, front, rear = [triangulate([ray(pixels[i,j], cameras[i]) for i in group]) for j in range(3)]
            forward = (front-rear)[:2]
            span = np.linalg.norm(forward)
            omitted = set(range(3)) - set(group)
            error = next((np.linalg.norm(project(tip, cameras[i])-tips[i]) for i in omitted), '')
            w.writerow([variant, '+'.join(names[i] for i in group), (tip-front)[:2] @ forward / span,
                        tip[2], span, error])

# Rescore the original held-out spins with coupled height and longitudinal offsets.
sys.path.insert(0, str(here.parent / 'recorded-pose-inset'))
from camera_spins import LENS_X_MM, turn_centres

def pivots(height):
    spins = turn_centres(here.parent / 'camera-spins' / 'camera-picks.csv', height=height)['spins']
    return np.array([spin['centre_x_mm'] for spin in spins[-2:]])

with (here / 'shift.csv').open() as handle:
    rows = list(csv.DictReader(handle))
predictions = np.array([[float(next(r['pivot_error_mm'] for r in rows if
    float(r['shift_mm']) == shift and r['spin'] == spin)) for spin in ('ccw_2', 'cw_2')]
    for shift in (0, 1)]) + pivots(65.5)
with (here / 'landmark-pivot-scores.csv').open('w', newline='') as handle:
    w = csv.writer(handle)
    w.writerow(['source', 'tip_x_mm', 'height_mm', 'centered_rms_mm', 'shifted_rms_mm'])
    for file, label, key_x, key_z in [
            ('landmark-registration.csv','B','tip_x_mm','height_mm'),
            ('landmark-stereo.csv','A','tip_x_mm','height_mm'),
            ('repeat-landmark.csv','A repeat','tip_x_mm','tip_height_mm')]:
        with (here / file).open() as source:
            row = next(csv.DictReader(source))
        x, z = float(row[key_x]), float(row[key_z])
        observed = pivots(z) + x - LENS_X_MM
        w.writerow([label, x, z, *np.sqrt(np.mean((predictions-observed)**2, axis=1))])

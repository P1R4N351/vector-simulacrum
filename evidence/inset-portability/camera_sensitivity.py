"""Rescore frozen longitudinal pivots against the same fisheye image picks."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parent / 'recorded-pose-inset'))
from camera_spins import turn_centres

picks = here.parent / 'camera-spins' / 'camera-picks.csv'
spins = ('ccw_1', 'cw_1', 'ccw_2', 'cw_2')


def measured(focal, height):
    return np.array([r['centre_x_mm'] for r in turn_centres(picks, focal, height)['spins']])


with (here / 'shift.csv').open() as handle:
    errors = {(float(r['shift_mm']), r['spin']): float(r['pivot_error_mm'])
              for r in csv.DictReader(handle)}
nominal = measured(580, 65.5)
predicted = nominal + np.array([[errors[shift, spin] for spin in spins] for shift in (0, 1)])
fig, axes = plt.subplots(1, 2, figsize=(8, 3.8), sharey=True, layout='constrained')
with (here / 'camera-sensitivity.csv').open('w', newline='') as handle:
    writer = csv.writer(handle)
    writer.writerow(['focal_prior_px', 'height_mm', 'pair', 'centered_rms_mm',
                     'forward_rms_mm', 'observed_forward_offset_to_tie_mm'])
    for height in (62.5, 65.5, 68.5):
        gain = [[], []]
        for focal in (450, 580, 720):
            residual = predicted - measured(focal, height)
            for pair in (0, 1):
                a, b = residual[:, 2*pair:2*pair+2]
                rms = np.sqrt(np.mean([a*a, b*b], axis=1))
                tie = np.mean(a*a-b*b) / (2*np.mean(a-b))
                writer.writerow([focal, height, pair+1, *rms, tie])
                gain[pair].append(rms[0]-rms[1])
        for ax, values in zip(axes, gain):
            ax.plot([450, 580, 720], values, 'o-', label=f'{height} mm')
for ax, title in zip(axes, ('Selection pair', 'Held-out pair')):
    ax.axhline(0, color='gray', linewidth=0.8)
    ax.set(xlabel='Focal prior (pixels)', title=title)
axes[0].set_ylabel('Pivot RMS improvement (mm)')
axes[1].legend(title='Backpack lens height')
fig.savefig(here / 'camera-sensitivity.png', dpi=160)

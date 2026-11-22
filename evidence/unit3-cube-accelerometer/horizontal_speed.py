"""Interpolate docked horizontal vibration at each off-dock encoder speed."""
import csv
from itertools import product
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from lift_packet_transfer import read, scatter, strokes

root = Path(__file__).resolve().parent


def measurements(session):
    moves = read(root.parent / 'unit3-wheelstand' / session / 'moves.csv')
    speeds = {' '.join(row[k] for k in ('condition', 'direction', 'repeat')): float(row['speed_mm_s'])
              for row in moves if row['timing_valid'] == '1'}
    return [(name, direction, speeds[name], sum(pre), sum(active))
            for name, direction, pre, active, _ in strokes(session, scatter, horizontal=True)]


docked, off_dock = measurements('D1'), measurements('D2')
rows = []
fig, axes = plt.subplots(1, 2, figsize=(9, 4.5), layout='constrained')
for ax, direction in zip(axes, ('raise', 'lower')):
    groups = []
    for speed_group in ('slow', 'fast'):
        points = [(v, active-pre) for name, d, v, pre, active in docked
                  if d == direction and name.startswith(speed_group)]
        assert len(points) == 3
        groups.append(points)
    medians = np.median(groups, axis=1)
    ax.plot(*medians.T, color='teal', label='Docked median interpolation')
    for slow, fast in product(*groups):
        ax.plot(*np.array([slow, fast]).T, color='grey', alpha=.22)
    points = np.concatenate(groups)
    ax.scatter(*points.T, marker='o', facecolors='white', edgecolors='grey', label='Docked stroke')
    for name, d, speed, pre, active in off_dock:
        if d != direction:
            continue
        candidates = []
        for slow, fast in product(*groups):
            assert slow[0] < speed < fast[0]
            candidates.append(np.interp(speed, [slow[0], fast[0]], [slow[1], fast[1]]))
        estimate = np.interp(speed, *medians.T)
        observed = active-pre
        rows.append([name, speed, observed, estimate, min(candidates), max(candidates),
                     np.sqrt(pre), np.sqrt(active), np.sqrt(pre+estimate)])
        ax.scatter(speed, observed, marker='x', color='black', label='Off-dock observation' if name.endswith(' 1') else None)
    ax.set(title=direction.capitalize(), xlabel='Encoder 10–90% mean lift speed (mm/s)')
axes[0].set_ylabel('Horizontal acceleration variance above pre-stroke (mg²)')
axes[0].legend(fontsize=8)
fig.savefig(root/'horizontal-speed.png', dpi=160)
with (root/'horizontal-speed.csv').open('w') as stream:
    writer = csv.writer(stream)
    writer.writerow('stroke speed_mm_s observed_added_mg2 interpolated_added_mg2 min_pair_mg2 max_pair_mg2 quiet_mg observed_mg predicted_mg'.split())
    writer.writerows(rows)

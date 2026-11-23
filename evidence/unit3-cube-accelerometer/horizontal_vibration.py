"""Test whether rotation about gravity could account for lateral vibration differences."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from lift_packet_transfer import strokes

root = Path(__file__).resolve().parent


def horizontal_moment(packets):
    # Same mg² units and factor of two as the individual axis measurements.
    step = np.diff(packets, axis=1).reshape(-1, 2)
    return step.T @ step / (2 * len(step))


train = strokes('D1', horizontal_moment, horizontal=True)
test = strokes('D2', horizontal_moment, horizontal=True)
# Mean, rather than component-wise median, commutes with rotation.
excess = {d: np.mean([a-r for _, dr, r, a, _ in train if dr == d], axis=0)
          for d in ('raise', 'lower')}
rows = []
fig, axes = plt.subplots(1, 2, figsize=(9, 4), layout='constrained', sharey=True)
for ax, direction in zip(axes, ('raise', 'lower')):
    for index,(label, _, rest, active, gravity) in enumerate(t for t in test if t[1] == direction):
        observed = np.sqrt(np.trace(active))
        quiet = np.sqrt(np.trace(rest))
        prediction = np.sqrt(np.trace(rest + excess[direction]))
        trace_median = np.median([np.trace(a-r) for _,d,r,a,_ in train if d == direction])
        robust = np.sqrt(np.trace(rest) + trace_median)
        measured_excess = active - rest
        # Sorted eigenvalue differences are the minimum Frobenius mismatch
        # over rotations of a real symmetric 2x2 tensor.
        best = np.linalg.norm(np.linalg.eigvalsh(excess[direction]) - np.linalg.eigvalsh(measured_excess))
        fixed = np.linalg.norm(excess[direction] - measured_excess)
        rows.append([label, quiet, observed, prediction, np.trace(measured_excess),
                     np.trace(excess[direction]), fixed, best, robust])
        for value, marker, name, color in [(quiet,'_','Pre-stroke','gray'),(observed,'o','D2 measured','black'),
                                           (prediction,'x','D1 mean','teal'),(robust,'+','D1 median','C1')]:
            ax.plot(index, value, marker, color=color, label=name if index == 0 else None)
    ax.set(title=direction.capitalize(), xlabel='Timing-valid D2 stroke', xticks=range(index+1),
           xticklabels=[r[0].split()[-1] for r in rows if direction in r[0]])
axes[0].set_ylabel('Gravity-horizontal vibration (mg RMS)')
axes[1].legend()
fig.savefig(root/'horizontal-vibration.png', dpi=165)
with (root/'horizontal-vibration.csv').open('w') as stream:
    w=csv.writer(stream)
    w.writerow('stroke pre_mg measured_mg predicted_mg measured_added_mg2 predicted_added_mg2 fixed_residual_mg2 best_yaw_residual_mg2 robust_predicted_mg'.split())
    w.writerows(rows)

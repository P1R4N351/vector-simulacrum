"""Test two-slot versus adjacent-slot noise on two stationary shelf captures."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parent / 'unit3-cube-accelerometer'))
from stationary_noise import simulate, v


def ratio(packets):
    adjacent = np.mean(np.diff(packets, axis=1)**2, axis=(0, 1))
    outer = np.mean((packets[:, 2] - packets[:, 0])**2, axis=0)
    return np.sqrt(outer / adjacent)


def main():
    fig, axes = plt.subplots(1, 2, figsize=(8, 4), sharey=True, layout='constrained')
    rows = []
    for ax, name in zip(axes, ['stationary', 'stationary-repeat']):
        data = np.genfromtxt(here / f'{name}.csv.gz', names=True, delimiter=',')
        assert np.array_equal(data['slot'], np.tile([0, 1, 2], len(data)//3))
        measured = np.column_stack([data[c] for c in 'xyz']).reshape(-1, 3, 3)/8192
        force = measured.mean(axis=(0, 1))
        force /= np.linalg.norm(force)
        trials = []
        for seed in range(32):
            model = simulate(v['cube_accel_noise_density_ug'], force, seed,
                             v['cube_accel_word_format'])
            assert len(model) >= len(measured)
            trials.append(ratio(model[:len(measured)]))
        trials = np.array(trials)
        observed = ratio(measured)
        mean = trials.mean(axis=0)
        low, high = trials.min(axis=0), trials.max(axis=0)
        for axis, physical, predicted, lower, upper in zip('XYZ', observed, mean, low, high):
            rows.append([name, len(measured), axis, physical, predicted, lower, upper])
        ax.errorbar(range(3), mean, yerr=[mean-low, high-mean], fmt='s', capsize=4,
                    label='Sim · 32-seed range')
        ax.scatter(range(3), observed, color='#b94317', label='Recorded', zorder=3)
        ax.set(xticks=range(3), xticklabels=list('XYZ'), title=name)
    axes[0].set_ylabel('Two-slot / adjacent-slot RMS change')
    axes[1].legend(fontsize=8)
    fig.savefig(here / 'rest-lags.png', dpi=160)
    with (here / 'rest-lags.csv').open('w') as file:
        writer = csv.writer(file, lineterminator='\n')
        writer.writerow(['recording','packets','axis','recorded','sim_mean','sim_min','sim_max'])
        writer.writerows(rows)


if __name__ == '__main__':
    main()

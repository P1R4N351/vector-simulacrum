"""Compare the gravity-like direction in physical and simulated packet means."""
import csv
import sys

import matplotlib.pyplot as plt
import numpy as np

from yaw_transfer import cube_rotation, here, load_rows, run


def direction(v):
    return np.degrees(np.arctan2(v[:, 1], v[:, 2]))


def compare(name, fixture, ax):
    directory = here.parent / name
    zero = next(float(r['sec']) for r in load_rows('actions.csv', directory) if r['action'] == 'roll cube')
    raw = np.genfromtxt(directory / 'cube.csv.gz', delimiter=',', names=True)
    times = raw['sec'].reshape(-1, 3)[:, 0] - zero
    physical = np.column_stack([raw[k] for k in 'xyz']).reshape(-1, 3, 3).mean(axis=1) / 8192
    view = 'Before roll' if name == 'R8' else 'Next approach'
    yaw = next(float(r['grid_yaw_minus_sdk_deg']) for r in load_rows('optical-registration.csv', here)
               if r['calibration'] == 'Dock and first cube' and r['view'] == view)
    t, _, _, _, sensor = run(fixture, .00025, True, directory, cube_rotation(directory, yaw),
                             {'cube_chip_yaw_deg': 180, 'cube_accel_scale': [.970709]*3},
                             articulated=True, shelf_support=True)
    st = np.array([ts for ts, _ in sensor.messages])
    sv = np.array([np.frombuffer(payload[2:], '<i2').reshape(3, 3).mean(axis=0)
                   for _, payload in sensor.messages]) / 8192
    # Reference both instruments to their own starting face, including the sideways holdout.
    def relative(a, baseline):
        return (direction(a) - np.median(direction(a[baseline])) + 180) % 360 - 180
    real = relative(physical, (times >= -1) & (times < 0))
    sim = relative(sv, (st > .1) & (st < 1))
    ax.plot(times, real, '.', ms=3, label='Physical packet mean')
    ax.plot(st, sim, lw=1, label='Shelf-supported replay')
    ax.axvline(t[-1], ls='--', color='.4', label='Chassis input ends')
    ax.set(xlim=(2.8, 5.7), ylabel='Y–Z direction change (°)', title=name)
    scores = []
    for start, end in [(4.6, 5.), (5., 5.208), (5.208, 5.27)]:
        keep = (times >= start) & (times < end)
        other = (st >= start) & (st < end)
        if keep.any():
            scores.append([name, start, end, keep.sum(), np.median(real[keep]),
                           np.median(np.linalg.norm(physical[keep], axis=1)),
                           np.median(sim[other]) if other.any() else None])
    return scores


if __name__ == '__main__':
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), layout='constrained')
    with (here / 'roll-onset.csv').open('w') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(['run','start_s','end_s','physical_packets','median_direction_deg',
                         'median_packet_norm_g','sim_median_direction_deg'])
        for name, ax in zip(('R8', 'R9'), axes):
            writer.writerows(compare(name, sys.argv[1], ax))
    axes[0].axvspan(5.46, 5.66, color='green', alpha=.15, label='Camera final appearance')
    axes[0].legend(fontsize=8)
    axes[1].set_xlabel('Seconds from roll command')
    fig.savefig(here / 'roll-onset.png', dpi=160)

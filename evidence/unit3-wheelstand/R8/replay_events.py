"""Evaluate frozen cube listeners against SDK events on the native shelf roll."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[2]))
from simulacrum.cube import MovementListener, TapListener, UpAxisListener, engine_samples, packet


def read_rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))


start = next(float(row['sec']) for row in read_rows(here/'actions.csv') if row['action'] == 'roll cube')
delay = float(read_rows(here.parents[1]/'unit3-physical-repeats/tap-receipt-transfer.csv')[0]['prior_delay_s'])
stream = np.genfromtxt(here/'cube.csv.gz', delimiter=',', names=True).reshape(-1, 3)
taps, move, axis = TapListener(), MovementListener(), UpAxisListener()
initial_axis = None
for group in stream:
    assert np.array_equal(group['slot'], [0, 1, 2])
    assert np.all(group['tap'] == group['tap'][0]) and np.all(group['sec'] == group['sec'][0])
    payload = packet(int(group['tap'][0]), np.column_stack([group[k] for k in 'xyz']).astype(int))
    t = group['sec'][0] - start
    taps.receive(t, payload)
    for sample in engine_samples(payload):
        move.update(t, sample)
        axis.update(t, sample)
    if initial_axis is None:
        initial_axis = axis.axis
sdk = read_rows(here/'events.csv')
series = {
    'Counter change': taps.events,
    'Counter + prior delay': [t + delay for t in taps.events],
    'SDK tap': [float(r['sec']) - start for r in sdk if r['event'] == 'object_tapped'],
    'Replay moving': [t for t, moving in move.events if moving],
    'SDK moving': [float(r['sec']) - start for r in sdk if r['event'] == 'object_moved'],
    'Replay up-axis change': [t for t, _ in axis.events],
    'SDK up-axis change': [float(r['sec']) - start for r in sdk if r['event'] == 'object_up_axis_changed'],
}
fig, ax = plt.subplots(figsize=(8, 3.5), layout='constrained')
last = stream[-1]['sec'][0] - start
with (here/'event-replay.csv').open('w') as f:
    writer = csv.writer(f, lineterminator='\n')
    writer.writerow(['series', 'count', 'seconds_after_roll'])
    for i, (label, times) in enumerate(series.items()):
        writer.writerow([label, len(times), ' '.join(f'{t:.6f}' for t in times)])
        ax.scatter(times, [i]*len(times), marker='|', s=180)
ax.axvline(last, ls='--', c='.4', label='Last recorded cube packet')
ax.set(yticks=range(len(series)), yticklabels=list(series), xlabel='Seconds after roll command',
       title='Cube packets replayed through frozen event listeners')
ax.invert_yaxis()
ax.legend(loc='lower left', fontsize=8)
fig.savefig(here/'event-replay.png', dpi=150)
print('first and last axis codes:', initial_axis, axis.axis, 'last packet:', last)

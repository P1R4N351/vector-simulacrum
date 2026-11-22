"""Compare reported face changes with packet replay and distance to a face boundary."""
import csv
import gzip
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[1]))
from simulacrum.cube import UpAxisListener, engine_samples, packet


def margin(vector):
    """Angular distance to the nearest equal-dominance plane, in degrees."""
    magnitudes = sorted(abs(vector))
    distance = (magnitudes[-1] - magnitudes[-2]) / np.sqrt(2)
    return np.degrees(np.arcsin(distance / np.linalg.norm(vector)))


summary = []
traces = {}
for path in sorted((here.parent / 'unit3-wheelstand').rglob('cube.csv.gz')):
    events_path = path.with_name('events.csv')
    if not events_path.exists():
        continue
    with gzip.open(path, 'rt') as f:
        samples = list(csv.DictReader(f))
    if not samples:
        continue
    with events_path.open() as f:
        sdk = [row for row in csv.DictReader(f) if row['event'] == 'object_up_axis_changed']
    listener = UpAxisListener()
    trace = []
    initial = None
    assert len(samples) % 3 == 0
    for offset in range(0, len(samples), 3):
        group = samples[offset:offset+3]
        assert [int(s['slot']) for s in group] == [0, 1, 2]
        assert len({s['sec'] for s in group}) == 1
        words = [[int(s[k]) for k in 'xyz'] for s in group]
        time = float(group[0]['sec'])
        for accel in engine_samples(packet(int(group[0]['tap']), words)):
            listener.update(time, accel)
            if listener.axis:
                if initial is None:
                    initial = listener.axis
                trace.append((time, margin(listener.filtered)))
    run = path.parent.name
    traces[run] = np.array(trace)
    summary.append([run, len(samples)//3, initial, listener.axis, len(listener.events),
                    len(sdk), min(m for _, m in trace)])

with (here / 'sdk-axis-transfer.csv').open('w') as f:
    writer = csv.writer(f, lineterminator='\n')
    writer.writerow(['run', 'packets', 'first_axis', 'last_axis', 'replay_changes',
                     'logged_changes', 'nearest_boundary_deg'])
    writer.writerows(summary)
fig, (ax, detail) = plt.subplots(2, 1, figsize=(9, 6), layout='constrained')
ax.bar([r[0] for r in summary], [r[-1] for r in summary])
ax.set(ylabel='Closest approach (°)', title='Distance of filtered acceleration to a face-change boundary')
run = min(summary, key=lambda r: r[-1])[0]
detail.plot(*traces[run].T, linewidth=1)
detail.axhline(0, color='black', linewidth=.7)
detail.set(xlabel='Recording time (s)', ylabel='Angular margin (°)', title=f'{run} · closest recorded approach')
fig.savefig(here / 'sdk-axis-transfer.png', dpi=150)

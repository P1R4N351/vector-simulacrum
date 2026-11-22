"""Check the calm-mode bit and transfer run S's loop duration to active cube events."""
import csv
import gzip
from bisect import bisect_left
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
root = here.parent / 'unit3-wheelstand'
intervals = []
events = []
for path in sorted(root.rglob('events.csv')):
    state_path = path.with_name('state.csv.gz')
    if not state_path.exists():
        continue
    with gzip.open(state_path, 'rt') as f:
        states = [(float(r['sec']), int(r['status'])) for r in csv.DictReader(f)]
    times = [t for t, _ in states]
    with path.open() as f:
        rows = [r for r in csv.DictReader(f) if r['event'] in
                ('object_tapped','object_moved','object_stopped_moving','object_up_axis_changed')]
    # SDK callbacks can be delivered out of stamp order. Compare distinct payloads
    # in robot time order and require local status coverage at both callbacks.
    accepted = {}
    for row in rows:
        t = float(row['sec'])
        i = bisect_left(times, t)
        if not 0 < i < len(times):
            continue
        before, after = states[i-1:i+1]
        if after[0] - before[0] > .3 or (before[1] ^ after[1]) & 0x400:
            continue
        calm = bool(before[1] & 0x400)
        events.append((path.parent.name, row['event'], calm))
        accepted[int(row['timestamp'])] = (i, calm)
    ordered = sorted(accepted)
    for a,b in zip(ordered,ordered[1:]):
        ia, mode = accepted[a]
        ib, other = accepted[b]
        dt = b-a
        if mode != other or dt > 1000:
            continue
        lo, hi = sorted((ia,ib))
        if any(bool(status & 0x400) != mode for _,status in states[lo-1:hi+1]):
            continue
        ticks = 50 if mode else 6
        period = 254 / 50 * ticks
        n = round(dt / period)
        intervals.append((path.parent.name,mode,a,b,dt,n,dt-n*period,dt-round(dt/(5*ticks))*5*ticks))
with (here/'sdk-clock-mode.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['run','calm','start_ms','end_ms','interval_ms','periods','residual_run_s_ms','residual_nominal_ms'])
    w.writerows(intervals)
fig, ax = plt.subplots(figsize=(8,4), layout='constrained')
for run in sorted({r[0] for r in intervals}):
    group=[r for r in intervals if r[0]==run]
    ax.scatter([r[4] for r in group],[r[6] for r in group],label=run)
ax.axhline(0,color='.5',lw=1)
ax.set(xlabel='Difference between distinct robot timestamps (ms)', ylabel='Residual to run S clock (ms)',
       title='Cube callbacks with bracketing SDK power-mode samples')
ax.legend(ncol=3)
fig.savefig(here/'sdk-clock-mode.png',dpi=150)
print('bracketed events',len(events),'calm',sum(e[2] for e in events))
print('intervals',len(intervals),'residuals transfer / nominal (median, max)',
      [np.percentile([abs(r[i]) for r in intervals],[50,100]) for i in (6,7)])

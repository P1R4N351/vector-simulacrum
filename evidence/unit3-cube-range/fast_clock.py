"""Bound the observed proximity changes under two candidate acquisition periods."""
from pathlib import Path
import csv
import json

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
periods = np.array([.03072, .06144])


def observable_ceiling(t, period):
    # The acquisition number can change only when the phase crosses a sample.
    # Check each open phase interval; exact boundaries share a neighbouring result.
    edges = np.unique(np.r_[0., np.remainder(t, period), period])
    phases = (edges[1:] + edges[:-1]) / 2
    ids = np.floor((t[:, None] - phases) / period)
    return int(np.max(np.count_nonzero(np.diff(ids, axis=0), axis=0)))


rows = []
for run in "gjklmoq":
    path, = here.parent.glob(f"hw-rec-*/run-{run}-*.jsonl")
    records = [json.loads(line) for line in path.read_text().splitlines()]
    states = [r for r in records if r.get('k') == 's' and 'proxq' in r]
    # Wheels are a proxy for tread activity; actual drive power was not logged.
    groups = []
    for r in states:
        speed = max(abs(r['wl']), abs(r['wr']))
        mode = 'turning or driving' if speed >= 30 else 'stationary' if speed == 0 else None
        if r['status'] & 4096:
            mode = None
        if not groups or mode != groups[-1][0] or r['t']-groups[-1][1][-1]['t'] > .2:
            groups.append((mode, []))
        groups[-1][1].append(r)
    for mode, group in groups:
        if not mode:
            continue
        # Disjoint 17-report windows, with 16 observable transitions.
        for start in range(0, len(group)-16, 17):
            batch = group[start:start+17]
            t = np.array([r['t'] for r in batch])
            pairs = [(r['prox'], r['proxq']) for r in batch]
            count = sum(a != b for a,b in zip(pairs,pairs[1:]))
            # An additional upper bound ignores packet jitter inside a window.
            # Within end-to-end duration T there can be at most ceil(T/P) refreshes.
            ceilings = [observable_ceiling(t-t[0], p) for p in periods]
            rows.append((run.upper(), mode, t[0], t[-1]-t[0], count, *ceilings,
                         int(np.ceil((t[-1]-t[0]) / periods[1])),
                         (batch[-1]['img_ts']-batch[0]['img_ts']) / 1000, np.min(np.diff(t))))
with (here/'fast-clock.csv').open('w') as f:
    w = csv.writer(f, lineterminator='\n')
    w.writerow(('run','mode','start_s','duration_s','observed_changes','fast_ceiling',
                'mid_ceiling','mid_elapsed_ceiling','image_clock_elapsed_s','min_host_gap_s'))
    w.writerows(rows)
fig, axes = plt.subplots(1,2, figsize=(9,4), sharey=True, layout='constrained')
for mode, ax in zip(('stationary','turning or driving'), axes):
    group = [r for r in rows if r[1] == mode]
    for r in group:
        ax.plot([r[5],r[6]], [r[4],r[4]], color='#557f91', alpha=.2)
    x, y = np.array([(r[6],r[4]) for r in group]).T
    vals, counts = np.unique(np.c_[x,y], axis=0,return_counts=True)
    ax.scatter(*vals.T, s=18+8*counts, color='#156783')
    ax.plot([8,16],[8,16],'--',color='0.45')
    ax.set(title=f'{mode.capitalize()} · {len(group)} windows',xlabel='61.44 ms observable ceiling',
           xlim=(7.5,16.5),ylim=(-.6,16.7))
    print(mode, len(group), 'exceeds fast, mid, elapsed-mid',
          [sum(r[4]>r[i] for r in group) for i in (5,6,7)])
axes[0].set_ylabel('Observed changes in 16 transitions')
fig.savefig(here/'fast-clock.png', dpi=160)

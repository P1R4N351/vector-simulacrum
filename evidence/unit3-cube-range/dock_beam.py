"""Transfer the frozen printed-fork ray sweep and approach delay to archived dock motion."""
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
beam = np.genfromtxt(here / 'fork-beam.csv', names=True, delimiter=',', dtype=None, encoding='utf-8')
beam = beam[beam['fork'] == 'printed']
fig, (trace, bars) = plt.subplots(1, 2, figsize=(11, 4), layout='constrained')
results = []
for run in 'gjklmoq':
    path, = here.parent.glob(f'hw-rec-*/run-{run}-*.jsonl')
    states = [json.loads(line) for line in path.read_text().splitlines()]
    states = [row for row in states if row.get('k') == 's' and 'proxq' in row]
    t, h, dist, quality = np.array([[r[k] for r in states] for k in ('t', 'lift', 'prox', 'proxq')])
    dock = np.array([bool(r['status'] & 4096) and r['wl'] == r['wr'] == 0 for r in states])
    in_band = dock & (h >= 34) & (h <= 56)
    # Age of this exact reported distance/quality pair, a lower bound on repeat duration.
    changes = np.r_[True, (np.diff(dist) != 0) | (np.diff(quality) != 0)]
    age = t - np.maximum.accumulate(np.where(changes, t, -np.inf))
    for delay in (0, .071):
        height = np.interp(t-delay, t, h)
        idx = np.abs(height[:, None] - beam['lift_mm']).argmin(axis=1)
        majority = beam['ray_hits'][idx] >= 10
        floor = dist == 30
        tp, tn, fp, fn = [np.count_nonzero(in_band & mask) for mask in
                          (majority & floor, ~majority & ~floor, majority & ~floor, ~majority & floor)]
        results.append([run.upper(), int(delay*1000), tp, tn, fp, fn,
                        np.count_nonzero(in_band & (age >= .25)),
                        np.count_nonzero(in_band & changes),
                        np.count_nonzero(in_band & changes & (majority == floor))])
    if run == 'j':
        keep = (t >= t[in_band][0] - .3) & (t <= t[in_band][-1] + .3)
        trace.plot(t[keep], dist[keep], 'o-', ms=2.5, label='Reported range (mm)')
        trace.plot(t[keep], h[keep], label='Lift height (mm)')
        trace.scatter(t[keep & changes], dist[keep & changes], marker='x', s=50,
                      color='black', label='Distance or quality changed')
        trace.set(title='Run J on dock', xlabel='Recording time (s)', ylabel='Millimetres')
        trace.legend(fontsize=8)
with (here/'dock-beam.csv').open('w') as f:
    w = csv.writer(f, lineterminator='\n')
    w.writerow(['run','delay_ms','true_floor','true_longer','false_floor','missed_floor','pair_age_ge250ms','changed_pair','correct_on_changed_pair'])
    w.writerows(results)
for i, delay in enumerate((0,71)):
    rows = [r for r in results if r[1] == delay]
    bars.bar(np.arange(len(rows))+(i-.5)*.35, [(r[2]+r[3])/sum(r[2:6])*100 for r in rows], .35,
             label=f'{delay} ms')
bars.set(xticks=np.arange(len(rows)), xticklabels=[r[0] for r in rows], ylim=(0,100),
         xlabel='Archived recording', ylabel='Correct floor / longer (%)', title='Frozen majority-of-rays rule')
bars.legend()
fig.savefig(here/'dock-beam.png', dpi=160)

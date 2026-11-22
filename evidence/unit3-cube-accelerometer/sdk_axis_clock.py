"""Check SDK payload clock granularity on archived run S."""
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
with (here.parent / 'hw-rec-20261121/run-s-hold-cube.jsonl').open() as f:
    rows = [r for r in map(json.loads, f) if 'timestamp' in r]
faces = [r for r in rows if r['event'] == 'object_up_axis_changed']
# Estimate a single tick from successively reported face changes in the first burst.
train = np.diff([r['timestamp'] for r in faces if r['t'] < 72])
period = np.median(train[(train > 0) & (train < 300)])
out = []
for event in sorted({r['event'] for r in rows}):
    subset = [r for r in rows if r['event'] == event]
    for a, b in zip(subset, subset[1:]):
        receipt = 1000 * (b['t'] - a['t'])
        if receipt > 3000:
            continue
        payload = b['timestamp'] - a['timestamp']
        ticks = round(payload / period)
        fit = event == 'object_up_axis_changed' and b['t'] < 72
        out.append((event, b['t'], fit, receipt, payload, ticks, payload - ticks * period))
with (here / 'sdk-axis-clock.csv').open('w') as f:
    writer = csv.writer(f, lineterminator='\n')
    writer.writerow(['event','end_s','fit_burst','receipt_interval_ms','payload_interval_ms',
                     'nearest_ticks','residual_ms'])
    writer.writerows(out)
fig, ax = plt.subplots(figsize=(8, 4.2), layout='constrained')
for event in sorted({r[0] for r in out}):
    pairs = [r for r in out if r[0] == event and not r[2]]
    ax.scatter([r[3] for r in pairs], [r[4] for r in pairs], label=event.removeprefix('object_'))
for tick in np.arange(0, 3200, period):
    ax.axhline(tick, color='gray', alpha=.2, lw=.7)
ax.set(xlabel='Time between SDK callbacks on recorder (ms)', ylabel='Difference in robot timestamps (ms)',
       title=f'Run S · held-out event intervals against a {period:g} ms clock')
ax.legend()
fig.savefig(here / 'sdk-axis-clock.png', dpi=150)
holdout = [abs(r[-1]) for r in out if not r[2]]
print(period,len(holdout),np.median(holdout),max(holdout))
print([(round(r[3],1),r[4]) for r in out if r[0]=='object_up_axis_changed' and not r[4]])

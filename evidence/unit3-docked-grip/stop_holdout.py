"""Evaluate the four previously published pulse predictions on the second session."""
import csv
import gzip
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent
with gzip.open(root / 'stop-holdout.jsonl.gz', 'rt') as stream:
    records = [json.loads(line) for line in stream]
states = [r for r in records if r['kind'] == 'state']
events = [r for r in records if r['kind'] == 'command']
replies = {r['n']:r['t'] for r in records if r['kind'] == 'response'}
assert len(events) == 18 and len(replies) == 12
assert all(s['status'] & 4096 and not s['left'] and not s['right'] for s in states)
assert any(r.get('docked') for r in records if r['kind'] == 'end')
with (root/'stop-clock.csv').open() as stream:
    frozen = {r['anchor']: (float(r['predicted_clear_s'])-float(r['shift_s']),
                            float(r['predicted_set_s'])-float(r['shift_s'])) for r in csv.DictReader(stream)}

# Estimate phase only from the first session. Each clear/set edge is interval
# censored, so use the median of the eight midpoint estimates of pulse onset.
with (root/'stop-clock.csv').open() as stream:
    train = [r for r in csv.DictReader(stream) if r['anchor']=='dispatch']
duration = frozen['direct hold + dispatch'][1] - frozen['direct hold + dispatch'][0]
onsets = []
for r in train:
    onsets.append((float(r['last_set_s']) + float(r['first_clear_s'])) / 2)
    onsets.append((float(r['last_clear_s']) + float(r['first_set_s'])) / 2 - duration)
phase = np.median(onsets)
frozen['direct hold + first-session phase'] = phase, phase+duration
print('first-session phase',phase)
fig, axes = plt.subplots(4, 3, figsize=(12, 9), layout='constrained', sharex=True, sharey=True)
panels = iter(axes.flat)
rows = []
for e in events:
    samples = [s for s in states if -.1 <= s['t'] - e['t'] <= .45]
    t = np.array([s['t'] - e['t'] for s in samples])
    clear = np.array([not bool(s['status'] & 256) for s in samples])
    h = np.array([s['lift'] for s in samples])
    if e['action'] == 'sham':
        rows.append([e['n'], 'sham', 'control', len(t), sum(clear), 0, 0, sum(clear), np.ptp(h)])
        continue
    ax = next(panels)
    ax.scatter(t*1000, 1-clear, s=14, color='black', label='SDK samples', zorder=5)
    for j,(name,(start,end)) in enumerate(frozen.items()):
        shift = replies[e['n']] - e['t'] if 'response' in name else 0
        a,b = start+shift,end+shift
        pred = (t>=a)&(t<b)
        rows.append([e['n'], 'stop', name, len(t), sum(clear), sum(clear&pred), sum(~clear&pred), sum(clear&~pred), np.ptp(h)])
        ax.plot(1000*np.array([-.1,a,a,b,b,.45]),np.array([1,1,0,0,1,1])+.022*(j-1.5),label=name)
    ax.set(title=f"Event {e['n']+1}",yticks=[0,1],yticklabels=['Clear','Set'])
fig.legend(*axes.flat[0].get_legend_handles_labels(), loc='outside upper center',ncol=3)
fig.supxlabel('Milliseconds from stop call (pod monotonic clock)')
fig.savefig(root/'stop-holdout.png',dpi=150)
with (root/'stop-holdout.csv').open('w') as stream:
    w=csv.writer(stream)
    w.writerow('event_zero_based action clock samples measured_clear clear_hits false_clear missed_clear lift_span_mm'.split())
    w.writerows(rows)
for name in [*frozen,'control']:
    selected=[r for r in rows if r[2]==name]
    print(name,np.sum([r[3:8] for r in selected],axis=0), 'height span',max(r[8] for r in selected))

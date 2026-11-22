"""Pair tap callbacks on the robot clock, then evaluate their host receipt timing."""
import csv
import gzip
import json
from pathlib import Path
import subprocess

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import linear_sum_assignment

here = Path(__file__).resolve().parent
root = here.parents[2]
paths = sorted(here.parent.glob('*/cube.csv.gz')) + sorted(here.glob('*/cube.csv.gz'))
results = json.loads(subprocess.check_output(['node',str(root/'web/tools/replay-cube-listeners.mjs'),
                                             *map(str,paths)],text=True))
with (here.parents[1]/'unit3-physical-repeats/tap-receipt-transfer.csv').open() as f:
    delay = float(next(csv.DictReader(f))['prior_delay_s'])

def match(radio, sdk, window):
    # A missing match costs more than every legal residual combined.
    gaps = np.abs(np.subtract.outer(radio,sdk))
    cost = np.where(gaps<=window, gaps, 1 + window*(1+len(radio)))
    i,j = linear_sum_assignment(cost)
    return {a:b for a,b in zip(i,j) if gaps[a,b]<=window}

summary, pairs = [], []
for path, events in zip(paths,results):
    with gzip.open(path,'rt') as f:
        data = [r for r in csv.DictReader(f) if r['slot']=='0']
    if not data: continue
    clock = {float(r['sec']):float(r['uptime']) for r in data}
    counter = [e['time_s'] for e in events if 'tap_count' in e]
    radio = [clock[t] for t in counter]
    with (path.parent/'events.csv').open() as f:
        sdk = [(float(r['timestamp'])/1000,float(r['sec'])) for r in csv.DictReader(f)
               if r['event']=='object_tapped' and min(clock.values())-.05<=float(r['timestamp'])/1000<=max(clock.values())+.05]
    times = [stamp for stamp,_ in sdk]
    paired = match(radio,times,.05)
    name = path.parent.name
    summary.append([name,len(counter),len(sdk),len(paired),len(match(radio,times,.1))])
    for i,j in paired.items():
        stamp,receipt = sdk[j]
        pairs.append([name,counter[i],receipt,1000*(stamp-radio[i]),1000*(receipt-counter[i]-delay)])
for file,header,rows in [
        ('browser-tap-counts.csv',['run','counter_changes','sdk_taps','paired_50ms','paired_100ms'],summary),
        ('browser-tap-times.csv',['run','packet_s','sdk_s','robot_clock_delta_ms','host_residual_ms'],pairs)]:
    with (here/file).open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(header);w.writerows(rows)
fig,ax=plt.subplots(figsize=(8,3.8),layout='constrained')
for idx,r in enumerate(summary):
    residual=[p[-1] for p in pairs if p[0]==r[0]]
    ax.scatter([idx]*len(residual),residual,alpha=.65,color='#22738a')
ax.axhline(0,color='.5',lw=1)
ax.set(xticks=range(len(summary)),xticklabels=[r[0] for r in summary],ylabel='SDK receipt minus prediction (ms)',
       title='Tap receipt timing with the earlier 145.906 ms delay')
fig.savefig(here/'browser-taps.png',dpi=150)
print(np.sum([r[1:] for r in summary],axis=0),np.percentile([p[-1] for p in pairs],[0,5,50,95,100]))

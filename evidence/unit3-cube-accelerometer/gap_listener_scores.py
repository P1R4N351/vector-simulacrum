"""Compare movement edges with stock SDK payload timestamps in collection order."""
from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt

root = Path(__file__).parent
def load(path):
    with path.open() as f:
        return list(csv.DictReader(f))

events = load(root/'gap-listener-events.csv')
runs = load(root/'gap-listener-runs.csv')
metrics = []
all_edges = []
for run in runs:
    name = run['run']
    variants = [[r for r in events if r['run']==name and r['mode']==mode] for mode in ('all','dedup')]
    for kind in set(r['kind'] for rows in variants for r in rows):
        assert sum(r['kind']==kind for r in variants[0])==sum(r['kind']==kind for r in variants[1])
    edges = [[r for r in rows if r['kind'] in ('start','stop')] for rows in variants]
    assert [r['kind'] for r in edges[0]] == [r['kind'] for r in edges[1]]
    all_edges.extend((a,b) for a,b in zip(*edges))
    sdk_path = root.parent/'unit3-wheelstand'/name/'events.csv'
    if not sdk_path.exists():
        continue
    sdk = [r for r in load(sdk_path) if r['event'] in ('object_moved','object_stopped_moving')]
    if not sdk:
        continue
    assert ['start' if r['event']=='object_moved' else 'stop' for r in sdk]==[r['kind'] for r in edges[0]], name
    for r,a,b in zip(sdk,*edges):
        stamp = int(r['timestamp'])
        errors = [round(float(x['uptime'])*1000)-stamp for x in (a,b)]
        origin = float(a['uptime'])-float(a['actual_elapsed_s'])
        clocks = [round((origin + float(a[k]))*1000)-stamp for k in ('nominal_elapsed_s','forecast_elapsed_s','anchored_elapsed_s')]
        metrics.append([name,a['kind'],stamp,*errors,*clocks])
with (root/'gap-listener-sdk.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['run','kind','sdk_ms','all_error_ms','dedup_error_ms','nominal_error_ms','forecast_error_ms','anchored_error_ms'])
    w.writerows(metrics)
errors=np.array([r[3:] for r in metrics])
print('packets,duplicates,events,raw movement,shifted movement',*[sum(int(r[k]) for r in runs) for k in ('packets','adjacent_duplicates','raw_events')],len(all_edges),sum(a['packet']!=b['packet'] for a,b in all_edges))
print('compared',len(errors),'MAE',np.abs(errors).mean(axis=0),'max',np.abs(errors).max(axis=0))
fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True,layout='constrained')
for ax,pair,scale,labels in zip(axes,((0,1),(2,3,4)),(1,1000),
                               (('Recorded receipts','Consecutive repeats skipped'),('30 ms clock','Resting pause forecast','First restart anchored'))):
    for i,label in zip(pair,labels):
        ax.plot(errors[:,i]/scale,'o--',label=label,alpha=.8)
    ax.axhline(0,color='.5',lw=.6)
    ax.legend()
axes[0].set_ylabel('Replay minus SDK (ms)')
axes[1].set_ylabel('Replay minus SDK (s)')
axes[1].set_xticks(range(len(metrics)),[r[0].split('/')[-1]+' '+r[1] for r in metrics],rotation=35,ha='right')
fig.savefig(root/'gap-listener-sdk.png',dpi=155)

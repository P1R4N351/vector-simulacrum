"""Evaluate frozen range thresholds on complete independent off-dock intervals."""
import csv
import numpy as np
import matplotlib.pyplot as plt
from cliff_pitch import here,replay

rows=[]
for path in sorted(here.parent.glob('*/actions.csv'))+[here/'S22/actions.csv']:
    folder=path.parent
    if folder.name in ('W4','W8'): continue
    with path.open() as f: events=list(csv.DictReader(f))
    starts=[float(e['sec']) for e in events if e['action']=='undock' and e['kind']=='end' and "'ok': True" in e['response']]
    if not starts:continue
    stops=[float(e['sec']) for e in events if e['action'] in ('dock','disconnect') and e['kind']=='begin' and float(e['sec'])>starts[0]]
    if not stops:continue
    trace=replay(folder/'state.csv.gz',starts[0],(0,min(stops)-starts[0]))
    if not len(trace):continue
    real=trace[:,7].astype(bool); rays=trace[:,2:6]
    out=[folder.name,len(real),sum(real)]
    for threshold in (25,16.5):
        sim=((rays>threshold)|(rays<0)).any(axis=1)
        out.extend([sum(sim & ~real),sum(~sim & real)])
    rows.append(out)
with (here/'cliff-transfer.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n');w.writerow(['run','receipts','flagged','false_positive_25','false_negative_25','false_positive_16_5','false_negative_16_5']);w.writerows(rows)
x=np.arange(len(rows));counts=np.array([row[1:] for row in rows])
fig,ax=plt.subplots(layout='constrained',figsize=(8,4))
for i,(threshold,color) in enumerate([(25,'#42799b'),(16.5,'#ab5b34')]):
    xx=x+(i-.5)*.37
    ax.bar(xx,counts[:,2+2*i],.35,color=color,label=f'{threshold} mm extra flags')
    ax.bar(xx,counts[:,3+2*i],.35,bottom=counts[:,2+2*i],color=color,hatch='//',alpha=.65,label=f'{threshold} mm missed flags')
ax.set(xticks=x,xticklabels=[row[0] for row in rows],ylabel='SDK receipts with disagreement',title='Aggregate cliff flags on independent off-dock intervals')
ax.legend(fontsize=8);fig.savefig(here/'cliff-transfer.png',dpi=150)
print(rows,counts.sum(axis=0))

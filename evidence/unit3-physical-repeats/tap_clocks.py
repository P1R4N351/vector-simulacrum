"""Compare the payload clock against every counter transition, including unmatched edges."""
from pathlib import Path
import csv
import gzip
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
def read(path):
    with (gzip.open if path.suffix == '.gz' else open)(path,'rt') as f:
        return list(csv.DictReader(f))

fig, axes = plt.subplots(3,1,figsize=(10,7),layout='constrained')
table=[]
for ax,run in zip(axes,('pickup','E','A')):
    if run=='pickup':
        root=here.parent/'unit3-cube-accelerometer'
        raw=[line.split() for line in (root/'pickup.tsv').read_text().splitlines()]
        packets=[(float(u),bytes.fromhex(h)) for host,u,h in raw]
        sdk=[int(r['robot_message_ms'])/1000 for r in read(root/'sdk-cube-events.csv') if r['event']=='object_tapped']
    else:
        packets=[(float(r['robot_uptime_s']),bytes.fromhex(r['payload_hex'])) for r in read(here/run/'radio.csv.gz')]
        sdk=[int(r['sdk_payload_ms'])/1000 for r in read(here/'sdk-tap-clocks.csv') if r['run']==run]
    previous=None
    transitions=[]
    for t,p in packets:
        if len(p)!=20 or p[0]!=2:
            continue
        if previous is not None and p[1]!=previous:
            transitions.append(t)
        previous=p[1]
    # Determine independently for each SDK stamp which radio change is closest.
    nearest=np.abs(np.subtract.outer(sdk,transitions)).argmin(axis=1)
    assert len(set(nearest))==len(sdk)
    for i,t in enumerate(transitions):
        loc=np.flatnonzero(nearest==i)
        delta=round((sdk[loc[0]]-t)*1000) if len(loc) else None
        gap=round((t-transitions[i-1])*1000) if i else None
        table.append([run,i+1,t,gap,delta])
        ax.plot(i+1,0,'|',ms=14,color='#aa6412')
        if loc.size: ax.scatter(i+1,delta,color='#236f77',s=30)
        else: ax.scatter(i+1,0,marker='x',s=55,color='#ab3232')
    ax.axhline(0,color='.8',zorder=0)
    ax.set(xticks=range(1,len(transitions)+1),ylabel='Offset (ms)',title=run,ylim=(-60,65))
axes[-1].set_xlabel('Radio counter transition in capture order')
fig.suptitle('SDK payload stamp relative to nearest radio change · crosses show unpaired changes')
fig.savefig(here/'tap-clocks.png',dpi=150)
with (here/'tap-clocks.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['run','transition','robot_uptime_s','previous_gap_ms','nearest_sdk_stamp_offset_ms'])
    w.writerows(table)

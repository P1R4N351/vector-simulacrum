"""Score packet-level tap events against two independent SDK time fields."""
import csv
import gzip
from pathlib import Path
import struct
import sys
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import linear_sum_assignment

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[1]))
from simulacrum.cube import TapListener

def read(path):
    with (gzip.open if path.suffix=='.gz' else open)(path,'rt') as f:
        return list(csv.DictReader(f))

delay=float(read(here.parent/'unit3-physical-repeats/tap-receipt-transfer.csv')[0]['prior_delay_s'])
pairs=[]
coverage=[]
for path in sorted((here.parent/'unit3-wheelstand').glob('*/cube.csv.gz')):
    data=read(path)
    if not data: continue
    run=path.parent.name
    a,b=TapListener(),TapListener()
    for i in range(0,len(data),3):
        rows=data[i:i+3]
        assert [r['slot'] for r in rows]==['0','1','2']
        assert len({r['tap'] for r in rows})==1
        packet=struct.pack('<BB9h',2,int(rows[0]['tap']),*(int(r[k]) for r in rows for k in 'xyz'))
        a.receive(float(rows[0]['uptime']),packet)
        b.receive(float(rows[0]['sec']),packet)
    sdk=[r for r in read(path.parent/'events.csv') if r['event']=='object_tapped']
    stamps=np.array([float(r['timestamp'])/1000 for r in sdk])
    cost=abs(np.subtract.outer(a.events,stamps))
    # Separate real matches from distant or absent callbacks. Any valid match
    # costs less than one dummy; invalid matches cost more than all dummies.
    cost=np.where(cost<=.100,cost,1e6)
    cost=np.column_stack((cost,np.full((len(a.events),len(a.events)),100.)))
    ri,ci=linear_sum_assignment(cost)
    matches={i:j for i,j in zip(ri,ci) if j<len(sdk)}
    coverage.append((run,len(data)//3,len(a.events),len(sdk),len(matches)))
    for i,u in enumerate(a.events):
        j=matches.get(i)
        pairs.append((run,i,u,b.events[i],None if j is None else 1000*(stamps[j]-u),
                      None if j is None else 1000*(float(sdk[j]['sec'])-b.events[i])))
with (here/'sdk-tap-transfer.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow('run packet_index_among_changes robot_uptime_s packet_host_s sdk_payload_offset_ms sdk_host_offset_ms'.split())
    w.writerows(pairs)
with (here/'sdk-tap-coverage.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow('run packets listener_events sdk_events matched_within_100ms'.split());w.writerows(coverage)
matched=[p for p in pairs if p[-1] is not None]
host=np.array([p[-1] for p in matched])
payload=np.array([p[-2] for p in matched])
fig,ax=plt.subplots(figsize=(9,4),layout='constrained')
ax.scatter(range(len(host)),host,label='SDK callback receipt minus packet receipt')
ax.scatter(range(len(host)),payload,label='SDK payload stamp minus robot packet receipt')
ax.axhline(delay*1000,ls='--',color='.35',label='Earlier callback delay')
ax.set(xticks=range(len(host)),xticklabels=[p[0] for p in matched],ylabel='Offset (ms)',xlabel='Matched taps in recording order')
ax.legend(fontsize=8)
fig.savefig(here/'sdk-tap-transfer.png',dpi=160)
print(np.sum([r[1:] for r in coverage],axis=0), 'payload range',payload.min(),payload.max(),
      'median host',np.median(host),'median absolute errors',np.median(abs(host)),np.median(abs(host-delay*1000)),
      'max corrected',max(abs(host-delay*1000)), 'unmatched',[p[:3] for p in pairs if p[-1] is None])

"""Compare transmitted tap counts and retained Z steps in complete packet triplets."""
import csv
import gzip
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[1]))
from simulacrum.cube import TAP_THRESH, int16

events=[]
totals=[]
for path in sorted((here.parent/'unit3-wheelstand').glob('*/cube.csv.gz')):
    with gzip.open(path,'rt') as f:
        rows=list(csv.DictReader(f))
    if not rows:
        continue
    slots=np.asarray([int(r['slot']) for r in rows]).reshape(-1,3)
    assert np.all(slots==[0,1,2])
    count=np.asarray([int(r['tap']) for r in rows]).reshape(-1,3)
    assert np.all(count==count[:,0,None])
    t=np.asarray([float(r['uptime']) for r in rows[::3]])
    z=np.asarray([int(r['z']) for r in rows]).reshape(-1,3)
    dt=np.diff(t)
    assert np.all(dt>=0)
    incr=np.diff(count[:,0])%256
    run=path.parent.name
    for long in (False,True):
        mask=(dt>.15)==long
        totals.append((run,long,mask.sum(),dt[mask].sum(),sum(incr[mask]>0),sum(incr[mask])))
    for j in np.flatnonzero(incr):
        changes=[abs(int16(int(v))) for v in np.diff(np.r_[z[j,-1],z[j+1]])]
        events.append((run,t[j],t[j+1],dt[j],incr[j],max(changes),max(changes[1:])))
with (here/'tap-gap-events.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow('run previous_receipt_s new_receipt_s interval_s counter_increment largest_boundary_or_internal_z_step_counts largest_internal_z_step_counts'.split())
    w.writerows(events)
with (here/'tap-gap-exposure.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow('run long_gap intervals elapsed_s changed_packets count_increments'.split())
    w.writerows(totals)
fig,ax=plt.subplots(figsize=(8,4),layout='constrained')
for j,r in enumerate(events):
    ax.scatter(r[3]*1000,r[6]/8192,s=30,alpha=.65,color='tab:blue')
ax.axhline(TAP_THRESH/8192,ls='--',color='tab:orange',label='Firmware Z-step threshold')
ax.axvline(150,color='gray',ls=':',label='Long receipt interval')
ax.set(xlabel='Interval between counter observations (ms)',ylabel='Largest internal Z step in new packet (g)')
ax.legend()
fig.savefig(here/'tap-gap.png',dpi=160)
for long in (False,True):
    print(long,np.sum([r[2:] for r in totals if r[1]==long],axis=0))
    e=[r for r in events if (r[3]>.15)==long]
    print('without internal threshold',sum(r[6]<=TAP_THRESH for r in e),'/',len(e),'max increment',max((r[4] for r in e),default=0))

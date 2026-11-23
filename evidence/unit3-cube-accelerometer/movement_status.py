"""Check SDK broadcast proxies against contemporaneous chassis status."""
import csv
import gzip
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[1]))
from simulacrum.cube import MovementListener,engine_samples,packet

def read(path):
    with (gzip.open if path.suffix=='.gz' else open)(path,'rt') as f:
        return list(csv.DictReader(f))

results=[]
for collection in ('unit3-wheelstand','unit3-physical-repeats'):
    wheel=collection=='unit3-wheelstand'
    clock='sec' if wheel else 'receipt_s'
    for path in sorted((here.parent/collection).glob('*/cube.csv.gz')):
        data=read(path)
        if not data: continue
        base=path.parent
        state=read(base/('state.csv.gz' if wheel else 'robot.csv.gz'))
        st=np.array([float(r[clock]) for r in state])
        listener=MovementListener()
        for i in range(0,len(data),3):
            rows=data[i:i+3]
            assert [r['slot'] for r in rows]==['0','1','2']
            payload=packet(int(rows[0]['tap' if wheel else 'tap_count']),
                           [[int(r[k if wheel else k+'_counts']) for k in 'xyz'] for r in rows])
            j=np.searchsorted(st,float(rows[0][clock]),side="right")-1
            carrying=j>=0 and bool(int(state[j]["status"])&2)
            for v in engine_samples(payload):
                listener.update(float(rows[0][clock]),v,handling_cube=carrying)
        sdk=[(float(r[clock]),r['event']=='object_moved') for r in read(base/('events.csv' if wheel else 'object-events.csv'))
             if r['event'] in ('object_moved','object_stopped_moving')]
        used=set()
        for t,moving in listener.events:
            idx=np.searchsorted(st,t,side='right')
            if idx==0 or idx==len(st): raise ValueError('State does not bracket event')
            prev,nxt=state[idx-1:idx+1]
            matches=[(abs(u-t),j) for j,(u,kind) in enumerate(sdk) if j not in used and kind==moving and abs(u-t)<=.30]
            match=None
            if matches:
                _,match=min(matches)
                used.add(match)
            results.append((base.name,round(t,4),moving,match is not None,int(prev['status']),int(nxt['status']),
                            round(t-st[idx-1],4),round(st[idx]-t,4)))
        assert len(listener.broadcast_events)==len(sdk)
        assert all(any(flag==s and abs(u-t)<=.30 for t,s in sdk) for u,flag in listener.broadcast_events)
        assert len(used)==len(sdk),(base.name,sdk,used)
with (here/'movement-status.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow('run host_s moving sdk_match prev_status next_status prev_age_s next_delay_s'.split())
    w.writerows(results)
fig,ax=plt.subplots(figsize=(9,4),layout='constrained')
for i,(name,mask) in enumerate([('Listener',0),('Carry bit',2),('Pick/place bit',4),('Either bit',6)]):
    pred=np.array([not (r[4]&mask) for r in results])
    sdk=np.array([r[3] for r in results])
    tp=int(np.sum(pred&sdk));fp=int(np.sum(pred&~sdk));fn=int(np.sum(~pred&sdk))
    ax.bar(i-.22,tp,width=.22,color='#167368',label='Matched SDK' if i==0 else None)
    ax.bar(i,fp,width=.22,color='#ae4d25',label='Extra' if i==0 else None)
    ax.bar(i+.22,fn,width=.22,color='#7663ad',label='Missed SDK' if i==0 else None)
    print(name,tp,fp,fn)
ax.set(xticks=range(4),xticklabels=['Listener','Carry bit','Pick/place bit','Either bit'],ylabel='Movement transitions')
ax.legend();fig.savefig(here/'movement-status.png',dpi=150)
for r in results:print(r[:4],r[4]&6,r[5]&6)

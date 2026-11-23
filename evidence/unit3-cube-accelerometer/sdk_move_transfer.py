"""Replay movement hysteresis from observed accelerometer packets across runs."""
import csv
import gzip
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[1]))
from simulacrum.cube import MovementListener, engine_samples, packet
records=[]
fig,axes=plt.subplots(4,1,figsize=(8,9),layout='constrained')
focus=dict(zip(('R5','R8','R10','R11'),axes))
for path in sorted((here.parent/'unit3-wheelstand').glob('*/cube.csv.gz')):
    with gzip.open(path,'rt') as f:
        rows=list(csv.DictReader(f))
    if not rows: continue
    run=path.parent.name
    listener=MovementListener()
    scores=[]
    for i in range(0,len(rows),3):
        triplet=rows[i:i+3]
        assert [r['slot'] for r in triplet]==['0','1','2']
        u=float(triplet[0]['uptime'])
        payload=packet(int(triplet[0]['tap']),[[int(r[k]) for k in 'xyz'] for r in triplet])
        for sample in engine_samples(payload):
            listener.update(u,sample)
        scores.append((u,listener.score))
    with (path.parent/'events.csv').open() as f:
        sdk=[(int(r['timestamp'])/1000,r['event']=='object_moved') for r in csv.DictReader(f)
             if r['event'] in ('object_moved','object_stopped_moving')]
    t,signal=np.array(scores).T
    for state in (True,False):
        pred=[u for u,s in listener.events if s==state]
        obs=[u for u,s in sdk if s==state]
        # There is at most one SDK event of each type in each archived capture.
        assert len(obs)<=1
        nearest=min(pred,key=lambda u:abs(u-obs[0])) if pred and obs else None
        for u in pred or [None]:
            matched=u is not None and u==nearest and abs(u-obs[0])<=.25
            records.append((run,state,3*len(scores),int(u is not None),int(matched),u if u is not None else '',
                            obs[0] if matched else '',1000*(u-obs[0]) if matched else '',t[-1],signal.max()))
        if obs and (nearest is None or abs(nearest-obs[0])>.25):
            records.append((run,state,3*len(scores),0,1,'',obs[0],'',t[-1],signal.max()))
    if run in focus:
        ax=focus[run]
        ref=sdk[0][0] if sdk else listener.events[0][0]
        ax.plot(t-ref,signal,label='Listener score')
        for value in (5000,10000): ax.axhline(value,color='.6',lw=.6)
        for u,state in sdk:
            ax.axvline(u-ref,color='tab:orange',ls='-' if state else '--',label='SDK start' if state else 'SDK stop')
        ax.set(xlim=(-1.5,13 if run=='R5' else 2),ylabel=run)
        ax.legend(fontsize=8)
axes[-1].set_xlabel('Seconds from first SDK start (R5: first listener start)')
fig.savefig(here/'sdk-move-transfer.png',dpi=160)
with (here/'sdk-move-transfer.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow('run moving samples listener_events sdk_events predicted_uptime sdk_uptime prediction_minus_sdk_ms last_packet_uptime peak_score'.split())
    w.writerows(records)
for r in records:
    if any(r[3:5]): print(r)

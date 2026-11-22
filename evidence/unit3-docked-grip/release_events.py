"""Compare SDK broadcasts with replay of measured and simulated cube packets."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from face_contact import start_raise
from simulacrum.cube import MovementListener, TapListener, engine_samples, packet

root=Path(__file__).resolve().parent
with (root/'release-events.csv').open() as stream:
    records=list(csv.DictReader(stream))
meta=[r for r in records if r['kind']=='packet']
sdk=[r for r in records if r['kind']!='packet']
slots=np.loadtxt(root/'cube.csv',delimiter=',',skiprows=1).reshape(-1,3,5)
assert len(slots)==len(meta) and np.all(slots[:,:,1]==[0,1,2])
move,tap=MovementListener(),TapListener()
scores=[]
pred=[]
for row,group in zip(meta,slots):
    host,u=map(float,(row['host_seconds'],row['robot_uptime_s']))
    assert abs(group[0,0]-host)<1e-5
    payload=packet(int(row['tap_count']),group[:,2:].astype(int).tolist())
    n=len(tap.events)
    tap.receive(u,payload)
    if len(tap.events)>n: pred.append(('object_tapped',host,u))
    for sample in engine_samples(payload):
        prev=move.moving
        move.update(u,sample)
        if move.moving!=prev:
            pred.append(('object_moved' if move.moving else 'object_stopped_moving',host,u))
    scores.append((host,move.score))
fig,axes=plt.subplots(2,1,figsize=(8,5),layout='constrained',sharex=True)
scores=np.array(scores)
axes[0].plot(*scores.T,'.-',ms=3)
for threshold in (5000,10000): axes[0].axhline(threshold,lw=.7,color='gray')
axes[0].set_ylabel('Movement listener score')
rows=[]
levels={'object_tapped':0,'object_moved':1,'object_stopped_moving':2}
delay=.145906
for kind,y in levels.items():
    p=[(h,u) for k,h,u in pred if k==kind]
    s=[r for r in sdk if r['kind']==kind]
    print(kind,p,[(r['host_seconds'],r['robot_uptime_s']) for r in s])
    assert len(p)==len(s)
    for (h,u),r in zip(p,s):
        receipt=float(r['host_seconds'])
        rows.append([kind,h,receipt,1000*(float(r['robot_uptime_s'])-u),1000*(receipt-h),1000*(receipt-h-delay) if kind=='object_tapped' else ''])
        axes[1].plot([h,receipt],[y,y],color='gray')
        axes[1].scatter(h,y,color='tab:blue',marker='x')
        axes[1].scatter(receipt,y,facecolors='none',edgecolors='tab:orange')
for gap in (0,1,2):
    sim,_=start_raise(gap)
    start=sim.data.time
    sim.advance(2.45)
    print('free replay',gap,[t-start for t in sim.cube.taps.events if t>=start],
          [(t-start,m) for t,m in sim.cube.movement.broadcast_events if t>=start])
axes[1].scatter([],[],color='tab:blue',marker='x',label='Recorded packet replay')
axes[1].scatter([],[],facecolors='none',edgecolors='tab:orange',label='SDK callback')
axes[1].legend(fontsize=8)
axes[1].set(xlabel='Pod receipt seconds relative to lift command',yticks=list(levels.values()),yticklabels=['Tap','Moved','Stopped'],xlim=(-.23,2.5))
fig.savefig(root/'release-events.png',dpi=165)
with (root/'release-event-scores.csv').open('w') as stream:
    w=csv.writer(stream);w.writerow('kind packet_host_s sdk_host_s sdk_stamp_minus_packet_uptime_ms sdk_minus_packet_host_ms tap_delay_residual_ms'.split());w.writerows(rows)

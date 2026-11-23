"""Replay pickup with the photographed cube's SDK carry ID and bracketing states."""
import bisect
import csv
import sys
from pathlib import Path
import matplotlib.pyplot as plt
from decode import decode
here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[1]))
from simulacrum.cube import MovementListener,engine_samples

def read(name):
    with (here/name).open() as f:
        return list(csv.DictReader(f))
state=read('robot.csv')
times=[float(r['pod_unix_s']) for r in state]
carry=[int(r['carry_id']) for r in state]
actions=read('action-times.csv')
zero=float(next(r['pod_unix_s'] for r in actions if r['action']=='pickup'))
pickup_end=float(next(r['pod_unix_s'] for r in actions if r['action']=='pickup' and r['event']=='end'))
listener=MovementListener()
target_proxy=MovementListener()
trace=[]
events=[]
for ns,u,count,slots in decode(here/'pickup.tsv')[0]:
    t=ns*1e-9
    i=bisect.bisect_right(times,t)-1
    assert 0<=i<len(times)-1
    n=len(listener.events)
    for accel in engine_samples(bytes([2,count])+slots):
        listener.update(t,accel,handling_cube=carry[i]==1)
        target_proxy.update(t,accel,handling_cube=carry[i]==1 or zero<=t<pickup_end)
    trace.append((t-zero,listener.score))
    for event in listener.events[n:]:
        events.append((t-zero,u,event[1],carry[i],carry[i+1],t-times[i],event in listener.broadcast_events,event in target_proxy.broadcast_events))
sdk=[(float(r['pod_unix_s'])-zero,float(r['robot_message_ms'])/1000,r['event']=='object_moved')
     for r in read('sdk-cube-events.csv') if r['event'] in ('object_moved','object_stopped_moving')]
with (here/'release-broadcast.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow('sec robot_uptime_s moving preceding_carry_id following_carry_id preceding_age_s carry_only_broadcast pickup_target_proxy_broadcast matched_sdk_host_delta_ms'.split())
    for row in events:
        matches=[(t-row[0])*1000 for t,u,m in sdk if m==row[2] and abs(u-row[1])<.1]
        w.writerow((*row,matches[0] if matches else ''))
fig,ax=plt.subplots(figsize=(9,4),layout='constrained')
ax.plot(*zip(*trace),color='#45638a',label='Movement score')
ax.fill_between([t-zero for t in times],0,20000,where=[v==1 for v in carry],step='post',alpha=.16,label='Carry ID 1')
for row in events:
    ax.scatter(row[0],10000 if row[2] else 5000,marker='o' if row[6] else 'x',color='#af5425')
for t,u,m in sdk:
    ax.axvline(t,color='#23783f',ls='--',label='SDK stop')
ax.set(xlim=(-1,15),xlabel='Seconds from pickup command',ylabel='Movement score')
ax.legend();fig.savefig(here/'release-broadcast.png',dpi=160)
print(events,sdk)

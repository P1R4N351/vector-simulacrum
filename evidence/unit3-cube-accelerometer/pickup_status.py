"""Recover state-bit edges from the original pickup stream and test a task-aware proxy."""
import bisect
import csv
import json
import sys
from pathlib import Path
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
def read(name):
    with (here/name).open() as f:
        return list(csv.DictReader(f))
actions=read('action-times.csv')
start=float(next(r['pod_unix_s'] for r in actions if r['action']=='pickup'))
decoder=json.JSONDecoder()
states={}
with open(sys.argv[1]) as f:
    for line in f:
        line=line.strip()
        while line:
            obj,n=decoder.raw_decode(line)
            line=line[n:].strip()
            if obj['kind']=='robot_state':
                states[obj['t']]=obj['data']
published=read('robot.csv')
assert len(states)==len(published)
for row in published:
    assert states[float(row['pod_unix_s'])]['carrying_object_id']==int(row['carry_id'])
times=sorted(states)
edges=[]
for t in times:
    status=states[t]['status']
    values=(bool(status&2),bool(status&4),states[t]['carrying_object_id'])
    if not edges or values!=edges[-1][1:]:
        edges.append((t-start,*values))
with (here/'pickup-status-edges.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n');w.writerow(('sec','carrying','picking_or_placing','carry_id'));w.writerows(edges)
pickup_end=float(next(r['pod_unix_s'] for r in actions if r['event']=='end' and r['action']=='pickup'))-start
with (here/'pickup-status-events.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('sec','status_before','status_after','targeted_pick_or_carry_broadcast','sdk_observed'))
    for r in read('release-broadcast.csv'):
        t=float(r['sec'])
        j=bisect.bisect_right(times,t+start)
        before,after=(states[k]['status'] for k in times[j-1:j+1])
        allow=not(before&2 or (0<=t<pickup_end and before&4))
        w.writerow((t,before,after,allow,bool(r['matched_sdk_host_delta_ms'])))
fig,ax=plt.subplots(figsize=(8,3.6),layout='constrained')
for bit,name,offset in [(2,'Carrying',0),(4,'Picking or placing',1.35)]:
    ax.step([t-start for t in times],[float(bool(states[t]['status']&bit))+offset for t in times],where='post',label=name)
for r in read('release-broadcast.csv'):
    ax.axvline(float(r['sec']),color='tab:green' if r['matched_sdk_host_delta_ms'] else '.4',ls='--')
ax.set(xlim=(-.5,14),xlabel='Seconds after pickup command',yticks=[0,1,1.35,2.35],yticklabels=['off','on','off','on'])
ax.legend();fig.savefig(here/'pickup-status.png',dpi=160)
print([e for e in edges if -1<e[0]<14])

with (here/'targeted-pick-transfer.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('run','sec','active_action','prior_status','proxy_broadcast','sdk_observed'))
    for row in read('movement-status.csv'):
        run=row['run'];t=float(row['host_s'])
        physical=run=='A'
        group='unit3-physical-repeats' if physical else 'unit3-wheelstand'
        with (here.parent/group/run/'actions.csv').open() as stream:
            history=[a for a in csv.DictReader(stream) if float(a['receipt_s' if physical else 'sec'])<=t]
        last=history[-1]
        action=last['action'] if last['event' if physical else 'kind']=='begin' else ''
        st=int(row['prev_status'])
        allow=not (st&2 or (action.startswith('pickup') and st&4))
        w.writerow((run,t,action,st,allow,row['sdk_match']))
        assert allow==(row['sdk_match']=='True')

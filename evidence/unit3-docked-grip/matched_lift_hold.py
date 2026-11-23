"""Find cube radio packets from physically stationary, comparable-height lift holds."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent
target = 45.42
rows = []
for path in sorted((root.parent/'unit3-wheelstand').rglob('cube.csv.gz')):
    states = path.with_name('state.csv.gz')
    if not states.exists():
        continue
    state = np.genfromtxt(states, delimiter=',', names=True)
    radio = np.genfromtxt(path, delimiter=',', names=True)
    if len(radio)<3 or len(state)<3:
        continue
    slots = radio.reshape(-1,3)
    assert np.all(slots['slot']==[0,1,2]) and np.all(slots['sec']==slots['sec'][:,:1])
    with path.with_name('actions.csv').open() as f:
        actions = list(csv.DictReader(f))
    for group in slots:
        t = group['sec'][0]
        lo, hi = np.searchsorted(state['sec'],[t-.1,t+.1])
        if lo==0 or hi==len(state):
            continue
        w = state[lo-1:hi+1]
        if (np.diff(w['sec']).max()>.15 or max(abs(w['lift']-target))>3 or
            np.ptp(w['lift'])>.2 or max(abs(w['left']).max(),abs(w['right']).max())>1):
            continue
        active = [r for r in actions if float(r['sec'])<=t]
        label = active[-1]['action'] if active and active[-1]['kind']=='begin' else 'idle'
        xyz=np.column_stack([group[k] for k in 'xyz'])
        steps=1000*abs(np.diff(np.linalg.norm(xyz/8192,axis=1)))
        rows.append([path.parent.name,t,np.median(w['lift']),label,
                     bool(np.any(w['status'].astype(int)&2)),*steps])
with (root/'matched-lift-hold.csv').open('w') as f:
    writer=csv.writer(f)
    writer.writerow(['run','receipt_s','lift_mm','active_action','any_carry_bit','step_01_mg','step_12_mg'])
    writer.writerows(rows)
dock=np.loadtxt(root/'cube.csv',delimiter=',',skiprows=1).reshape(-1,3,5)
hold=dock[(dock[:,0,0]>=1.15001)&(dock[:,0,0]<=1.37309)]
physical=np.abs(np.diff(np.linalg.norm(hold[:,:,2:]/8192,axis=2),axis=1))*1000
fig,ax=plt.subplots(figsize=(7,4),layout='constrained')
for label in sorted({r[0] for r in rows}):
    group=[r for r in rows if r[0]==label]
    ax.scatter(np.repeat([r[2] for r in group],2),np.array([r[-2:] for r in group]).ravel(),
               label=label,alpha=.7)
ax.scatter(np.full(physical.size,target),physical.ravel(),marker='x',label='Failed hold')
ax.set(xlabel='Reported lift height (mm)',ylabel='Adjacent-slot acceleration magnitude change (mg)',yscale='log')
ax.legend(ncol=3,fontsize=8)
fig.savefig(root/'matched-lift-hold.png',dpi=165)
steps=np.array([r[-2:] for r in rows])
print(len(rows),np.median(steps),steps.max(),np.median(physical),{r[3] for r in rows},any(r[4] for r in rows))

"""Test an onboard acceleration predictor of vertical cube vibration on held-out D2."""
import csv
import gzip
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent


def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def observations(session):
    root = here.parent/'unit3-wheelstand'/session
    with gzip.open(root/'sdk-state.jsonl.gz','rt') as stream:
        states = [json.loads(line) for line in stream]
    t = np.array([s['sec'] for s in states])
    h = np.array([s['state']['lift_height_mm'] for s in states])
    # Magnitude is invariant to the head's instantaneous axis orientation.
    acc = np.array([[s['state']['accel'].get(k,0.) for k in 'xyz'] for s in states])
    magnitude = np.linalg.norm(acc,axis=1)/9.80665
    events = read(root/'actions.csv')
    packets = np.genfromtxt(root/'cube.csv.gz',names=True,delimiter=',').reshape(-1,3)
    assert np.all(packets['slot']==[0,1,2])
    z = packets['z']*1000/8192
    rows = []
    for move in read(root/'moves.csv'):
        if move['timing_valid']!='1':
            continue
        label = ' '.join(move[k] for k in ('condition','direction','repeat'))
        start = float(next(e['sec'] for e in events if e['kind']=='begin' and e['action']==label))
        rel = t-start
        base = np.median(h[(rel>=-.5)&(rel<0)])
        progress = (h-base)/float(move['travel_mm'])
        def crossing(level):
            i = np.flatnonzero((rel>=0)&(progress>=level))[0]
            return np.interp(level,progress[i-1:i+1],rel[i-1:i+1])
        lo,hi = crossing(.1),crossing(.9)
        masks = lambda tt: [(tt>=-.8)&(tt<-.1),(tt>=lo)&(tt<=hi)]
        robot = [magnitude[m] for m in masks(rel)]
        cube = [z[m] for m in masks(packets['sec'][:,0]-start)]
        assert len(cube[1])*3==int(move['slots'])
        rv, av = [np.var(a) for a in robot]
        cv, mv = [np.mean(np.diff(a,axis=1)**2)/2 for a in cube]
        rows.append(dict(session=session,stroke=label,direction=move['direction'],
                         robot_samples=len(robot[1]),robot_rest_mg=np.sqrt(rv),robot_active_mg=np.sqrt(av),
                         robot_excess=max(0,av-rv),cube_rest_var=cv,cube_active_var=mv))
    return rows


train = observations('D1')
test = observations('D2')
directional = {d:np.median([r['cube_active_var']-r['cube_rest_var'] for r in train if r['direction']==d])
               for d in ('raise','lower')}
# One nonnegative least-squares gain, no intercept; rest supplies the baseline.
x = np.array([r['robot_excess'] for r in train])
y = np.array([r['cube_active_var']-r['cube_rest_var'] for r in train])
gain = max(0.,x@y/(x@x))
# Test incremental information after accounting for raise/lower differences.
directions = np.array([r['direction'] for r in train])
mean_x = {d:x[directions==d].mean() for d in directional}
mean_y = {d:y[directions==d].mean() for d in directional}
dx = x - [mean_x[d] for d in directions]
dy = y - [mean_y[d] for d in directions]
conditional_gain = max(0., dx@dy/(dx@dx))
for r in train+test:
    d = r['direction']
    r['combined_mg'] = np.sqrt(max(0.,r['cube_rest_var']+mean_y[d]+conditional_gain*(r['robot_excess']-mean_x[d])))
    r['direction_mean_mg'] = np.sqrt(max(0.,r['cube_rest_var']+mean_y[d]))
    r['conditional_gain'] = conditional_gain
    r.update(gain=gain, measured_mg=np.sqrt(r['cube_active_var']),rest_mg=np.sqrt(r['cube_rest_var']),
             direction_mg=np.sqrt(r['cube_rest_var']+directional[r['direction']]),
             onboard_mg=np.sqrt(r['cube_rest_var']+gain*r['robot_excess']))
with (here/'onboard-vibration.csv').open('w',newline='') as stream:
    w=csv.DictWriter(stream,fieldnames=train[0].keys())
    w.writeheader()
    w.writerows(train+test)
fig,axes=plt.subplots(1,2,figsize=(10,3.8),layout='constrained',sharey=True)
for ax,rows,title in zip(axes,(train,test),('D1 · docked fit','D2 · off-dock test')):
    for key,marker in [('measured_mg','o'),('direction_mg','x'),('onboard_mg','+'),('combined_mg','1'),('rest_mg','_')]:
        ax.plot(range(len(rows)),[r[key] for r in rows],marker,label=key.split('_')[0])
    ax.set(title=title,xticks=range(len(rows)),xticklabels=[r['stroke'].replace('matched ','') for r in rows])
    ax.tick_params(axis='x',rotation=65)
    print(title, {key:round(np.mean([abs(r[key]-r['measured_mg']) for r in rows]),3)
                  for key in ('rest_mg','direction_mg','direction_mean_mg','onboard_mg','combined_mg')})
axes[0].set_ylabel('Cube Z within-packet scatter (mg)')
axes[1].legend(fontsize=8)
fig.savefig(here/'onboard-vibration.png',dpi=160)

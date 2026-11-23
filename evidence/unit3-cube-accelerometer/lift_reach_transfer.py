"""Predict cube vibration on withheld partial raises from full-stroke lift recordings."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
source = here.parent/'unit3-wheelstand'


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def full_strokes(session):
    folder = source/session
    moves = read(folder/'moves.csv')
    events = read(folder/'actions.csv')
    state = np.genfromtxt(folder/'state.csv.gz',names=True,delimiter=',')
    cube = np.genfromtxt(folder/'cube.csv.gz',names=True,delimiter=',')
    mag = np.linalg.norm(np.column_stack([cube[k] for k in 'xyz']),axis=1)*1000/8192
    rows = []
    for m in moves:
        if m['direction']!='raise' or m['timing_valid']!='1':
            continue
        label = ' '.join(m[k] for k in ('condition','direction','repeat'))
        start,end = [float(next(e['sec'] for e in events if e['action']==label and e['kind']==k)) for k in ('begin','end')]
        base = np.median(state['lift'][(state['sec']>=start-.5)&(state['sec']<start)])
        active = (cube['sec']>=start)&(cube['sec']<=end)
        height = np.interp(cube['sec'][active],state['sec'],state['lift'])
        rows.append((label,base,height,mag[active],float(m['travel_mm']),float(m['speed_mm_s']),
                     float(m['rest_mg']),float(m['moving_mg'])))
    return rows


train = full_strokes('D1')
length = np.array([r[4] for r in train])
speed = np.array([r[5] for r in train])
var = np.array([r[7]**2-r[6]**2 for r in train])
slope,offset = np.polyfit(speed,var,1)
tests = [(r[0],r[4],r[5],r[6],r[7]) for r in full_strokes('D2')]
tests.extend((r['run'],*[float(r[k]) for k in ('rise_mm','speed_mm_s','rest_sd_mg','rise_sd_mg')])
             for r in read(source/'R11/lift-vibration.csv'))
rows = []
for name,travel,rate,rest,meas in tests:
    # Match the slice of each D1 rise to the observed travel above its own starting height.
    slices = []
    for label,base,height,mag,_,_,idle,_ in train:
        select = (height>=base+.1*travel)&(height<=base+.9*travel)
        if np.sum(select)>=6:
            slices.append(np.var(mag[select])-idle**2)
    assert len(slices)>=3
    estimates = [0,np.median(var),offset+slope*rate,np.median(slices),
                 np.median(var)*(travel/np.median(length))**2]
    prediction = np.sqrt(rest**2+np.maximum(estimates,0))
    rows.append([name,travel,rate,rest,meas,len(slices),*prediction])
with (here/'lift-reach-transfer.csv').open('w',newline='') as f:
    w=csv.writer(f)
    w.writerow(['stroke','travel_mm','speed_mm_s','rest_mg','measured_mg','usable_D1_slices',
                'baseline_mg','full_mg','speed_mg','height_slice_mg','travel_scaled_mg'])
    w.writerows(rows)
fig,ax=plt.subplots(figsize=(9,4),layout='constrained')
for j,key in [(4,'Measured'),(6,'Pre-stroke'),(8,'Speed fit'),(9,'Matched height interval'),(10,'Travel-scaled amplitude')]:
    ax.plot(range(len(rows)),[r[j] for r in rows],'o' if j==4 else 'x-',label=key)
ax.set(xticks=range(len(rows)),xticklabels=[r[0].replace('matched ','D2 ') for r in rows],ylabel='Cube acceleration magnitude SD (mg)')
ax.legend(fontsize=8)
fig.savefig(here/'lift-reach-transfer.png',dpi=160)
for name,subset in [('D2',rows[:3]),('Partial native',rows[3:-1]),('R11',rows[-1:])]:
    print(name,[round(np.mean([abs(r[j]-r[4]) for r in subset]),3) for j in range(6,11)])

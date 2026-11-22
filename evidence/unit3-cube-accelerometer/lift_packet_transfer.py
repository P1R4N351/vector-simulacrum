"""Transfer within-packet XYZ vibration from docked D1 to off-dock D2 strokes."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def scatter(packets):
    return np.mean(np.diff(packets, axis=1)**2, axis=(0,1))/2


def strokes(session):
    root = here.parent/'unit3-wheelstand'/session
    events = read(root/'actions.csv')
    state = np.genfromtxt(root/'state.csv.gz',names=True,delimiter=',')
    groups = np.genfromtxt(root/'cube.csv.gz',names=True,delimiter=',').reshape(-1,3)
    assert np.all(groups['slot']==[0,1,2]) and np.all(groups['sec']==groups['sec'][:,:1])
    xyz = np.stack([groups[k] for k in 'xyz'],axis=-1)*1000/8192
    result = []
    for move in read(root/'moves.csv'):
        if move['timing_valid']!='1':
            continue
        label = ' '.join(move[k] for k in ('condition','direction','repeat'))
        start,end = [float(next(e['sec'] for e in events if e['action']==label and e['kind']==k)) for k in ('begin','end')]
        t = state['sec'] - start
        select = (t>=-.5)&(t<=end-start+.7)
        t, h = t[select],state['lift'][select]
        progress = (h-np.median(h[t<0]))/float(move['travel_mm'])
        def crossing(level):
            j = np.flatnonzero((t>=0)&(progress>=level))[0]
            return np.interp(level,progress[j-1:j+1],t[j-1:j+1])
        pt = groups['sec'][:,0]-start
        active = xyz[(pt>=crossing(.1))&(pt<=crossing(.9))]
        assert active.size//3==int(move['slots'])
        rest = xyz[(pt>=-.8)&(pt<-.1)]
        gravity = np.mean(rest, axis=(0,1))
        gravity /= np.linalg.norm(gravity)
        result.append((label,move['direction'],scatter(rest),scatter(active),gravity))
    return result


train = strokes('D1')
test = strokes('D2')
excess = {d: np.median([a-r for _,direction,r,a,g in train if direction==d],axis=0) for d in ('raise','lower')}
fig,axes = plt.subplots(1,3,figsize=(10,3.7),layout='constrained',sharey=True)
with (here/'lift-packet-transfer.csv').open('w',newline='') as stream:
    writer=csv.writer(stream)
    writer.writerow(['stroke','axis','rest_mg','measured_mg','predicted_mg','D1_added_variance_mg2','rest_gravity_axis_cosine'])
    for i,ax in enumerate(axes):
        for j,(label,d,r,a,g) in enumerate(test):
            quiet, measurement = np.sqrt(r[i]),np.sqrt(a[i])
            prediction = np.sqrt(max(0,r[i]+excess[d][i]))
            writer.writerow([label,'XYZ'[i],quiet,measurement,prediction,excess[d][i],g[i]])
            for value,marker,color,series in [(quiet,'_','grey','Pre-stroke'),(measurement,'o','black','D2 measured'),
                                              (prediction,'x','teal','D1 prediction')]:
                ax.plot(j,value,marker,color=color,label=series if j==0 else None)
        ax.set(title='Cube '+'XYZ'[i],xticks=range(len(test)),xticklabels=[a.replace('matched ','') for a,_,_,_,_ in test])
        ax.tick_params(axis='x',rotation=55)
axes[0].set_ylabel('Within-packet RMS step / √2 (mg)')
axes[1].legend(fontsize=8)
fig.savefig(here/'lift-packet-transfer.png',dpi=160)

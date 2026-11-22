"""Compare two physical contacts at the first sampled 5 degree pitch crossing."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
fig,axes=plt.subplots(3,1,figsize=(8,6),sharex=True,layout='constrained')
for name in ['W4','W8']:
    p=here/name
    commands=list(csv.DictReader((p/'actions.csv').open()))
    start=next(float(e['sec']) for e in commands if e['action']=='pop_a_wheelie')
    s=np.genfromtxt(p/'state.csv.gz',names=True,delimiter=',')
    c=np.genfromtxt(p/'cube.csv.gz',names=True,delimiter=',')
    pitch=np.degrees(s['pitch'])
    onset=s['sec'][(s['sec']>=start)&(pitch>5)][0]
    axes[0].plot(s['sec']-onset,pitch,'.-',label=name)
    axes[1].scatter(c['sec']-onset,np.linalg.norm(np.stack([c[k] for k in 'xyz']),axis=0)/8192,s=5)
    base=c['tap'][c['sec']>=start][0]
    axes[2].step(c['sec']-onset,c['tap']-base,where='post')
for ax,label in zip(axes,['Chassis pitch (degrees)','Cube specific force (g)','Tap increments']):
    ax.set(xlim=(-.8,1.0),ylabel=label)
axes[0].legend(); axes[-1].set_xlabel('Seconds from first SDK pitch sample above 5°')
fig.savefig(here/'W8/physical-repeat.png',dpi=155)

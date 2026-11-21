"""Zoom the W4 onboard state and cube packet measurements on wheelstand contact."""
import csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
here=Path(__file__).resolve().parent
actions=list(csv.DictReader((here/'actions.csv').open()))
zero=next(float(a['sec']) for a in actions if a['action']=='pop_a_wheelie')
r=np.genfromtxt(here/'state.csv.gz',delimiter=',',names=True)
c=np.genfromtxt(here/'cube.csv.gz',delimiter=',',names=True)
f,axes=plt.subplots(2,1,sharex=True,figsize=(8,4.8),layout='constrained')
axes[0].plot(r['sec']-zero,np.rad2deg(r['pitch']),'.-',ms=3,color='#286b8d')
axes[0].set_ylabel('Robot pitch (°)')
g=np.linalg.norm(np.stack([c[k] for k in 'xyz']),axis=0)/8192
axes[1].plot(c['sec']-zero,g,'.-',ms=3,lw=.7,color='#922c46',label='Cube accelerometer')
for t in c['sec'][1:][np.diff(c['tap'])>0]:
    axes[1].axvline(t-zero,color='.35',ls=':',lw=1)
axes[1].set(xlim=(3.3,4.85),ylabel='Specific force (g)',xlabel='Seconds after wheelstand command (packet receipt)')
f.savefig(here/'contact.png',dpi=165)

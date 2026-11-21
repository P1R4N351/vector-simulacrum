"""Locate firmware tap increments relative to solved contact forces in the nominal replay."""
from pathlib import Path
import csv

import matplotlib.pyplot as plt
import numpy as np

out=Path(__file__).resolve().parent/'E'
r=np.genfromtxt(out/'contact-load.csv',names=True,delimiter=',',dtype=None,encoding='utf8')
t=r['t_s']
increments=np.flatnonzero(np.diff(r['app_count'],prepend=r['app_count'][0]))
sent=np.flatnonzero(np.diff(r['reported_events'],prepend=r['reported_events'][0]))
fig,axes=plt.subplots(3,1,sharex=True,figsize=(10,6.4),layout='constrained')
for k,label in [('robot_normal_N','Robot on cube'),('floor_normal_N','Floor on cube')]:
    axes[0].plot(t,r[k],lw=.9,label=label)
axes[0].set_ylabel('Sum of normal loads (N)')
axes[0].legend()
axes[1].plot(t,r['chip_z_g'],lw=.8)
axes[1].set_ylabel('Chip Z before filtering (g)')
axes[2].plot(t,r['cube_tilt_deg'],lw=.9)
axes[2].set(ylabel='Cube tilt (°)',xlabel='Seconds after undock command',xlim=(50.49,50.94))
for ax in axes:
    for i in increments: ax.axvline(t[i],color='#963f17',alpha=.8,ls='--')
    for i in sent: ax.axvline(t[i],color='#586b9b',alpha=.8,ls=':')
fig.suptitle('Simulated load and cube rocking · dashed: app counter · dotted: packet report')
fig.savefig(out/'contact-load.png',dpi=155)
print('app',t[increments],'packet',t[sent])
for i in increments:
    m=(t>=t[i]-.030001)&(t<=t[i])
    print('preceding 30 ms',t[i],[(k,round(r[k][m].min(),4),round(r[k][m].max(),4)) for k in ['robot_normal_N','floor_normal_N','cube_tilt_deg','chip_z_g']])
for key in ['robot_normal_N','floor_normal_N']:
    load=r[key]>.000001
    j=np.flatnonzero(np.diff(load.astype(int)))+1
    print(key,[(round(t[a],3),round(t[b-1],3),bool(load[a])) for a,b in zip(np.r_[0,j],np.r_[j,len(t)]) if b-a>=3])

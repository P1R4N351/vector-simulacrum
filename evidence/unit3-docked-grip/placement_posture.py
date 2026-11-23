"""Check first-contact placement from above against the pre-command gravity direction."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent
samples = np.loadtxt(root/'cube.csv',delimiter=',',skiprows=1)
rest = samples[samples[:,0]<0,2:].mean(axis=0)
tilt = np.degrees(np.arctan2(np.linalg.norm(rest[:2]),rest[2]))
fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
for yaw,marker in [(-9,'x'),(0,'o'),(2,'+')]:
    with (root/f'pitched-hold-3mm-yaw{yaw}-from-above.csv').open() as stream:
        rows=[r for r in csv.DictReader(stream) if r['fork']=='Printed STL']
    pre,peak,force = np.array([[float(r[k]) for k in ('pre_tilt_deg','raised_peak_deg','mean_upward_fork_force_weights')] for r in rows]).T
    axes[0].scatter(pre,peak,marker=marker,label=f'{yaw}° yaw',alpha=.7)
    use=pre<2
    axes[1].scatter(100*force[use],peak[use],marker=marker,alpha=.7)
axes[0].axvline(tilt,ls=':',color='gray',label='Recorded resting force angle')
axes[0].set(xlabel='Pre-command inclination (°)',ylabel='Peak inclination during hold (°)',title='All 63 printed-fork placements')
axes[0].legend(fontsize=8)
axes[1].axvline(0,ls=':',color='gray')
axes[1].set(xlabel='Mean upward fork force (% of cube weight)',ylabel='Peak inclination during hold (°)',title='31 placements starting within 2° of upright')
fig.savefig(root/'placement-posture.png',dpi=165)

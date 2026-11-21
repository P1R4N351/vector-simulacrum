"""Gravity directions available for calibration in the shelf recordings."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
files = [(name,here/f'{name}.csv') for name in ('resting','undock-check','pickup')]
for name in ('A','B','C','D','E','F'):
    files.append((name,here.parent/'unit3-physical-repeats'/name/'cube.csv.gz'))
for name in ('W1','W2','W3','W4'):
    files.append((name,here.parent/'unit3-wheelstand'/name/'cube.csv.gz'))

fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
with (here/'orientation-coverage.csv').open('w') as out:
    w=csv.writer(out,lineterminator='\n')
    w.writerow(['session','second','samples','rms_g','mean_x_g','mean_y_g','mean_z_g','tilt_deg'])
    for name,path in files:
        a=np.genfromtxt(path,names=True,delimiter=',')
        if not a.size: continue
        keys=a.dtype.names
        tkey=next(k for k in ('robot_rx_uptime_s','receipt_s','sec') if k in keys)
        elapsed=a[tkey]-a[tkey][0]
        vectors=np.stack([a[k+'_counts' if k+'_counts' in keys else k] for k in 'xyz'],axis=1)/8192
        kept=[]
        for sec in range(int(elapsed[-1])):
            use=(elapsed>=sec)&(elapsed<sec+1)
            if sum(use)<45 or np.ptp(elapsed[use])<.75:continue
            v=vectors[use]
            m=v.mean(axis=0)
            rms=np.sqrt(np.mean(np.sum((v-m)**2,axis=1)))
            if rms>.012 or not .9<np.linalg.norm(m)<1.1:continue
            tilt=np.degrees(np.arctan2(np.linalg.norm(m[:2]),m[2]))
            w.writerow([name,sec,len(v),rms,*m,tilt])
            kept.append([*m,tilt])
        if kept:
            kept=np.asarray(kept)
            axes[0].scatter(kept[:,0],kept[:,1],s=8,label=name)
            axes[1].scatter(kept[:,3],np.linalg.norm(kept[:,:3],axis=1),s=8)
axes[0].set(xlabel='Mean X (g)',ylabel='Mean Y (g)',aspect='equal')
axes[0].legend(ncol=2,fontsize=7)
axes[1].set(xlabel='Apparent gravity tilt from +Z (degrees)',ylabel='Mean vector magnitude (g)')
fig.savefig(here/'orientation-coverage.png',dpi=155)

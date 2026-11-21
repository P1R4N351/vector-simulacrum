"""Resolve fork and shelf loads on W4's simulated cube and compare their times with packet receipts."""
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
here=Path(__file__).resolve().parent
rows=np.genfromtxt(here/'contact-loads.csv.gz',names=True,delimiter=',',dtype=None,encoding='utf8')
times,index=np.unique(rows['s'],return_inverse=True)
dt=np.median(np.diff(times))
def summed(source,column):
    mask=rows['source']==source
    return np.bincount(index[mask],weights=rows[column][mask],minlength=len(times))
fork=summed('fork','fz_N')
shelf=summed('shelf','fz_N')
yaw_f=summed('fork','yaw_Nm')
yaw_s=summed('shelf','yaw_Nm')
active=summed('fork','normal_N')>0
print('dt, active window, peak compression (fork and shelf), opposing vertical impulses:',dt,
      times[active][[0,-1]],-fork.min(),shelf.max(),dt*fork.sum(),dt*shelf[active].sum())
print('yaw impulses in contact:',dt*yaw_f.sum(),dt*yaw_s[active].sum())
fig,axes=plt.subplots(2,1,sharex=True,figsize=(9,5),layout='constrained')
axes[0].plot(times,-fork,label='Fork downward');axes[0].plot(times,shelf,'--',label='Shelf upward')
axes[0].set_ylabel('Simulated vertical load (N)');axes[0].legend()
axes[1].plot(times,np.cumsum(yaw_f)*dt,label='Fork');axes[1].plot(times,np.cumsum(yaw_s)*dt,'--',label='Shelf')
axes[1].set_ylabel('Yaw impulse (N m s)');axes[1].legend()
for ax in axes:
    for t in (3.983398,4.273996):ax.axvline(t,color='.25',ls=':',lw=1)
axes[1].set(xlabel='Seconds from wheelstand command · dotted lines are physical tap packet receipts',xlim=(3.88,4.34))
fig.savefig(here/'contact-balance.png',dpi=155)
np.savetxt(here/'contact-balance.csv',np.column_stack([times,fork,shelf,yaw_f,yaw_s]),delimiter=',',fmt='%.8g',header='sec,fork_vertical_N,shelf_vertical_N,fork_yaw_Nm,shelf_yaw_Nm',comments='')

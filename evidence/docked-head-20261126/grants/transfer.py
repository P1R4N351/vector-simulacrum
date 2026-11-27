"""Freeze a median-baseline displacement template on two sessions; transfer by grant time."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
root = Path(__file__).parent
grid = np.arange(0,8,.02)
def load(name):
    rows=list(csv.DictReader((root/(name+'.csv')).open()))
    grant=float(next(r for r in rows if r['event']=='granted')['receipt_s'])
    a=np.array([[float(r[k]) for k in ['receipt_s','head_deg','gyro_y']] for r in rows if r['event']=='state'])
    a[:,0]-=grant
    baseline=np.median(a[(a[:,0]>=-1)&(a[:,0]<0),1:],axis=0)
    return a,baseline
def profile(a,b):
    return np.column_stack([np.interp(grid,a[:,0],a[:,i+1]-b[i]) for i in range(2)])
train=[profile(*load(p)) for p in ['pilot','session-1']]
template=np.mean(train,axis=0)
a,b=load('session-2')
a=a[(a[:,0]>=0)&(a[:,0]<8)]
prediction=np.column_stack([np.interp(a[:,0],grid,template[:,i])+b[i] for i in range(2)])
fig,axes=plt.subplots(2,1,sharex=True,figsize=(9,5.6),layout='constrained')
for i,ax in enumerate(axes):
    ax.axhline(b[i],c='gray',ls=':',label='Stationary prediction')
    ax.plot(a[:,0],prediction[:,i],color='#ce791d',label='Prediction from two earlier grants')
    ax.plot(a[:,0],a[:,i+1],'.-',color='#226588',ms=3,label='Held-out physical trace')
    ax.set_ylabel(['Head angle (°)','Gyro Y (rad/s)'][i])
axes[0].legend(fontsize=8)
axes[-1].set_xlabel('Seconds after grant acknowledgement')
fig.savefig(root/'grant-transfer.png',dpi=160)
with (root/'grant-transfer-scores.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['window_s','packets','stationary_angle_rmse_deg','template_angle_rmse_deg','stationary_gyro_rmse_rad_s','template_gyro_rmse_rad_s'])
    for stop in [5.2,8]:
        mask=a[:,0]<stop
        errors=[np.sqrt(np.mean((arr[mask]-a[mask,1:])**2,axis=0)) for arr in [np.broadcast_to(b,prediction.shape),prediction]]
        w.writerow([stop,int(sum(mask)),*np.array(errors).T.ravel()])

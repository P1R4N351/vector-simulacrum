"""Infer forward travel from change in the apparent width of the stationary cube marker."""
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
here=Path(__file__).resolve().parent
source=here.parent/'unit3-cube-contact-replay'
states=np.genfromtxt(source/'inputs.csv',names=True,delimiter=',')
edges=np.genfromtxt(here/'edges.csv',names=True,delimiter=',')
def samples(i,lag):
    rows=states[states['trial']==i]
    q=edges[(edges['trial']==i)&(edges['seconds']<2.75)]
    widths=np.hypot(q['u1']-q['u0'],q['v1']-q['v0'])
    w0=np.median(widths[q['seconds']<.5])
    # Relative timing of SDK pose samples and host-receipt image timestamps.
    t=q['seconds']+1.5-lag
    theta=np.deg2rad(rows['yaw_deg'][0])
    travel=(rows['x_mm']-rows['x_mm'][0])*np.cos(theta)+(rows['y_mm']-rows['y_mm'][0])*np.sin(theta)
    return q['seconds'],1/w0-1/widths,np.interp(t,rows['sec'],travel)
def residual(x,ids):
    residuals=[]
    for i in ids:
        t,d,travel=samples(i,x[1])
        residuals.extend((d*x[0]-travel)[t>1.])
    return np.array(residuals)
fit=least_squares(residual,[10000.,.1],bounds=([4000,-.45],[16000,.45]),args=([1,2],),loss='soft_l1')
print('fit',fit.x)
fig,axs=plt.subplots(1,5,figsize=(14,3.3),sharey=True)
for i,ax in enumerate(axs,1):
    t,d,travel=samples(i,fit.x[1])
    _,_,orig=samples(i,0)
    static=537*23*d
    fittravel=fit.x[0]*d
    print(i,'initial, fitted RMSE',np.sqrt(np.mean((static[t>1]-orig[t>1])**2)),np.sqrt(np.mean((fittravel[t>1]-travel[t>1])**2)), 'n',sum(t>1))
    ax.plot(t,travel,'k.-',label='Odometry (aligned)')
    ax.plot(t,fittravel,'o-',color='#287c86',label='Calibrated optical')
    ax.plot(t,static,':',color='#cb663a',label='Previous optical scale')
    ax.set_title(f'Trial {i} · '+('fit' if i<=2 else 'held out'))
    ax.set_xlabel('Image receipt (s)')
axs[0].set_ylabel('Forward travel (mm)')
axs[-1].legend(loc='upper left',bbox_to_anchor=(1,1),frameon=False)
fig.tight_layout();fig.savefig(here/'approach.png',dpi=150)

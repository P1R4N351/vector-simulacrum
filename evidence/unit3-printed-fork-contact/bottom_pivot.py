"""Bound motion of any one material point on the bottom square between exposures."""
import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import lsq_linear
from optical_clearance import here, reconstruct, rows

half = 22.15
times = rows["sec"]
results = []
for lens,k,f in (("See3CAM",-1.499,473.11), ("pinhole",0,700)):
    for shelf in (False,True):
        poses = reconstruct(k,f,shelf)
        for (ra,ta,_),(rb,tb,_),a,b in zip(poses,poses[1:],times,times[1:]):
            # In the initial cube frame, bottom point displacement is D*p + delta.
            D = rb.as_matrix() - ra.as_matrix()
            constant = tb-ta-half*D[:,2]
            fit = lsq_linear(D[:,:2],-constant,bounds=(-half,half))
            volume = lsq_linear(D, ta-tb, bounds=(-half,half))
            top = lsq_linear(D[:,:2], ta-tb-half*D[:,2], bounds=(-half,half))
            results.append((lens,shelf,a,b,*fit.x,np.linalg.norm(fit.fun),
                            np.linalg.norm(tb-ta),np.linalg.norm(top.fun),np.linalg.norm(volume.fun)))
with (here/'bottom-pivot.csv').open('w') as file:
    w=csv.writer(file,lineterminator='\n')
    w.writerow(['lens','shelf','from_s','to_s','bottom_x_mm','bottom_y_mm','least_motion_mm','center_motion_mm','least_top_motion_mm','least_volume_motion_mm'])
    w.writerows(results)
fig,axes=plt.subplots(2,1,figsize=(8,6),layout='constrained',sharex=True)
for lens in ('See3CAM','pinhole'):
    for shelf in (False,True):
        r=[v for v in results if v[0]==lens and v[1]==shelf and v[3]<6.1]
        name=lens+(' shelf' if shelf else ' free')
        for ax,index in zip(axes,(6,8)):
            ax.plot([v[3] for v in r],[v[index] for v in r],'.-',label=name)
        print(name,'pre-drive max',max(v[6] for v in r if v[3]<0),'raise',[(round(v[3],2),round(v[6],2),round(v[7],2)) for v in r if v[3]>5])
axes[0].legend(ncol=2)
axes[0].set(ylabel='Least motion (mm)',title='Trial 1 · smallest motion of any bottom-face point between views')
axes[1].set(xlabel='Time of second exposure (s)',ylabel='Least top motion (mm)')
fig.savefig(here/'bottom-pivot.png',dpi=160)

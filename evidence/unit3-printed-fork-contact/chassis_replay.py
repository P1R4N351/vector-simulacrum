"""Apply each measured longitudinal discrepancy to the fork-and-links trajectory."""
import csv
import numpy as np
from evaluate import here,run,states

a=np.genfromtxt(here/'chassis-track.csv',names=True,delimiter=',')
idx=states['trial']==1
original=states['x_mm'][idx].copy()
with (here/'chassis-replay.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('trajectory','peak_tilt_deg','rightward_mm','endpoint_error_mm'))
    for label in ('odom_mm','vents_forward_mm','seam_forward_mm'):
        valid=np.isfinite(a[label])
        delta=a[label][valid]-a['odom_mm'][valid]
        stamp=a['sec'][valid]
        delta-=np.median(delta[stamp<0])
        states['x_mm'][idx]=original+np.interp(states['sec'][idx]-1.5,stamp,delta)
        result,_=run((1,'stl','overhead','weld',0,0,0),rails=True)
        w.writerow((label,result[-1],-result[9],result[-3]))
states['x_mm'][idx]=original

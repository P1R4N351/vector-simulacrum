import csv
from pathlib import Path
import numpy as np

here=Path(__file__).resolve().parent
with (here/'seam-results.csv').open('w') as out:
    w=csv.writer(out)
    w.writerow(['variant','start','contact','peak_deg','peak_to_flag_mm'])
    for variant in ['autonomous','seam']:
        for suffix,start in [('','direct'),('60','positive diagonal'),('-60','negative diagonal')]:
            r=np.genfromtxt(here/f'{variant}-driven{suffix}.csv',names=True,delimiter=',')
            on=np.flatnonzero(r['on'])
            if not on.size:
                w.writerow([variant,start,0,'',''])
                continue
            i=on[0]
            p=np.rad2deg(r['pitch_rad'])
            j=np.flatnonzero((p[:i]>-5)&(p[1:i+1]<=-5))[-1]+1
            peak=j+np.argmin(p[j:i+1])
            vel=(r['left_mm_s']+r['right_mm_s'])/2
            step=np.diff(r['time_s'])*(vel[1:]+vel[:-1])/2
            w.writerow([variant,start,1,p[peak],-step[peak:i].sum()])

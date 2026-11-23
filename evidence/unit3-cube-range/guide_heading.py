"""Compare reported pre-contact heading to the last observed dock, retaining the gyro check."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from release_optics import states

here=Path(__file__).resolve().parent
fig,(ax,paths,aim)=plt.subplots(1,3,figsize=(16,4.7),layout='constrained')
with (here/'mount-target-inputs.csv').open() as inputs,(here/'guide-heading.csv').open('w') as out:
    w=csv.writer(out,lineterminator='\n')
    w.writerow(['session','first_s','last_s','heading_start_deg','heading_end_deg','sdk_change_deg','gyro_z_integral_deg'])
    for obs in csv.DictReader(inputs):
        name=obs['session']
        s,i=states(name)
        t=np.array([float(r['receipt_s']) for r in s])
        dock=np.array([int(bool(int(r['status'])&4096)) for r in s])
        k=np.flatnonzero((np.diff(dock)==1)&(t[1:]>t[i]+3))[0]+1
        t-=t[k]
        a,b,c,d=[float(obs['q'+str(j)]) for j in range(4)]
        out_yaw=np.arctan2(-2*(a*d+b*c),-(1-2*(c*c+d*d)))
        err=np.array([float(r['yaw_rad']) for r in s])-out_yaw
        err=np.rad2deg(np.arctan2(np.sin(err),np.cos(err)))
        # End before the charger pose reset in all five sessions.
        window=(t>=-1.1)&(t<=-.12)
        times=t[window]; heading=err[window]
        gz=np.rad2deg([float(r['gz']) for r in s])[window]
        gyro=np.r_[0,np.cumsum(np.diff(times)*(gz[:-1]+gz[1:])/2)]
        line,=ax.plot(times,heading,'o-',ms=2,label=name)
        ax.plot(times,heading[0]+gyro,':',color=line.get_color())
        w.writerow([name,times[0],times[-1],heading[0],heading[-1],heading[-1]-heading[0],gyro[-1]])
ax.axhline(0,c='gray',lw=.8);ax.set(xlabel='Seconds from first charger report',ylabel='Heading relative to observed dock (°)',title='Recorded SDK yaw (solid), integrated head gyro (dotted)')
ax.legend(fontsize=8)
data=np.genfromtxt(here/'guide-entry.csv',delimiter=',',names=True,encoding='utf8',dtype=None)
for g in ['original','smooth','no_funnels','no_rails','no_guides']:
    rows=data[(data['guides']==g)&(data['heading_deg']==3)&(data['offset_mm']==0)]
    paths.plot(rows['x_mm']+120,rows['y_mm'],label=g.replace('_',' '))
paths.axvline(0,c='gray',lw=.8);paths.invert_xaxis();paths.legend(fontsize=8)
paths.set(xlabel='Chassis origin ahead of lip (mm)',ylabel='Lateral position (mm)',title='Simulated 3° entry from center, −30 mm/s')
with (here/'guide-entry-outcomes.csv').open('w') as out:
    w=csv.writer(out,lineterminator='\n')
    w.writerow(['guides','entry_yaw_deg','entry_y_mm','time_s','contact','end_yaw_deg','end_y_mm'])
    for g in np.unique(data['guides']):
        group=data[data['guides']==g]
        for h in np.unique(group['heading_deg']):
            subset=group[group['heading_deg']==h]
            for y in np.unique(subset['offset_mm']):
                r=subset[subset['offset_mm']==y][-1]
                w.writerow([g,h,y,r['time_s'],r['on'],r['yaw_deg'],r['y_mm']])
                if g=='original':
                    aim.scatter(h,y,color='#187551' if r['on'] else '#b4512e',marker='o' if r['on'] else 'x')
angles=np.linspace(-5,5,101)
aim.plot(angles,129.5*np.tan(np.deg2rad(angles)),':',c='gray',label='Projected pad path')
aim.scatter([],[],color='#187551',label='Acquired');aim.scatter([],[],c='#b4512e',marker='x',label='Miss')
aim.set(xlabel='Initial heading (°)',ylabel='Initial lateral offset (mm)',title='Driven rear treads, original guides')
aim.legend(fontsize=8)
fig.savefig(here/'guide-heading.png',dpi=150)

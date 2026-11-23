"""Wheel telemetry and approach travel from the fixed-speed and native docking trials."""
import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
def load(file):
    with file.open() as f:
        return list(csv.DictReader(f))

fig, (ax, ramp) = plt.subplots(1, 2, figsize=(11,4.5), layout='constrained')
scores = []
for name in 'ABCDE':
    with gzip.open(here.parent/'unit3-physical-repeats'/name/'robot.csv.gz', 'rt') as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r['receipt_s']) for r in rows])
    on = np.array([int(bool(int(r['status']) & 4096)) for r in rows])
    contact = np.flatnonzero((np.diff(on)==1) & (t[1:]>35))[0]+1
    t -= t[contact]
    v = np.array([(float(r['left_mmps'])+float(r['right_mmps']))/2 for r in rows])
    window = (t>=-1)&(t<=.2)
    ax.plot(t[window],v[window],'.-',color='#387b9b',alpha=.6,label='Recorded returns A–E' if name=='A' else None)
    scores.append((name,np.median(v[(t>=-1)&(t<=-.15)])))
for filename,label,color in [('mount-speed-before.csv','Canonical diagonal','#bd5525'),('mount-speed-driven.csv','Driven tread, direct','#347139')]:
    rows=load(here/filename)
    t=np.array([float(r['first_contact_rel_s']) for r in rows])
    v=np.array([(float(r['left_mmps'])+float(r['right_mmps']))/2 for r in rows])
    ax.plot(t,v,color=color,label=label)
    scores.append((label,np.median(v[(t>=-1)&(t<=-.15)])))
ax.set(xlabel='Seconds from charger contact',ylabel='Mean of left and right wheel speeds (mm/s)',title='Successful native mounts')
ax.legend(fontsize=8)
rows=load(here/'ramp-backup.csv')
for variant,label,style in [('canonical','Fixed guards','-'),('smooth_rear_guards','Low-friction guards',':'),('no_rear_guards','Rear guards disabled','--'),('driven_rear_envelope','Rear guards driven','-.')]:
    part=[r for r in rows if r['variant']==variant and r['speed_mmps']=='30' and r['offset_mm']=='2']
    ramp.plot([float(r['time_s']) for r in part],[float(r['origin_x_mm'])+120 for r in part],style,label=label)
ramp.axhline(0,color='gray',alpha=.55)
ramp.set(xlabel='Seconds from reverse command',ylabel='Chassis origin ahead of charger lip (mm)',title='Straight mount at −30 mm/s, 2 mm lateral offset')
ramp.legend(fontsize=8)
fig.savefig(here/'mount-speed.png',dpi=160)
with (here/'mount-speed-scores.csv').open('w') as f:
    writer=csv.writer(f,lineterminator='\n');writer.writerow(['return','median_mean_wheel_mmps']);writer.writerows(scores)

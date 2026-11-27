import csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

here = Path(__file__).resolve().parent
def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))
def col(rows, key):
    return np.array([float(r[key]) for r in rows])
def yaw(r, x, z):
    angle = col(r,'head_rad') + np.deg2rad(-.66)
    return np.sin(angle)*col(r,x) + np.cos(angle)*col(r,z)

fig, axes = plt.subplots(1, 2, figsize=(10,4),layout='constrained')
for label, file in [('Workshop 1','../states.csv'),('Workshop 2','../speed-holdout/states.csv'),
                    ('Workshop 3','../interleaved/states.csv'),('Driven rear rollers','autonomous-driven.csv')]:
    rows=read(here/file)
    if 'on' in rows[0]:
        on=col(rows,'on')>0
    else:
        rows=[r for r in rows if r['phase']=='return']
        on=col(rows,'flags').astype(int)&4096 != 0
    i=np.flatnonzero(on)[0]
    t=col(rows,'time_s')-float(rows[i]['time_s'])
    pitch=np.degrees(col(rows,'pitch_rad'))
    mask=(t>=-3.5)&(t<=.15)
    axes[0].plot(t[mask],pitch[mask],label=label)
    wheel=(col(rows,'left_mm_s')+col(rows,'right_mm_s'))/2
    print(label,'pitch at flag, median wheel speed',pitch[i],np.median(wheel[(t>=-.72)&(t<0)]))
axes[0].set(xlabel='Seconds from first charger flag',ylabel='SDK pitch (degrees)')
axes[0].legend(fontsize=8)
events=read(here.parent/'interleaved/events.csv')
real=read(here.parent/'interleaved/states.csv')
web=read(here.parent/'web-replay/browser.csv')+read(here/'roller-spins.csv')
for mode in ['vector-free-dock','passive','driven']:
    errors=[]
    speeds=[]
    for i,(cue,stop) in enumerate(zip([e for e in events if e['event']=='spin'],[e for e in events if e['event']=='stop'],strict=True),1):
        a,b=float(cue['time_s']),float(stop['time_s'])
        rr=[r for r in real if (a+b)/2<=float(r['time_s'])<b]
        ss=[r for r in web if r['scene']==mode and r['session']=='interleaved' and int(r['turn'])==i]
        pred=np.interp(col(rr,'time_s')-a,col(ss,'t_s'),yaw(ss,'gx','gz')).mean()
        errors.append(100*(pred/yaw(rr,'gyro_x','gyro_z').mean()-1))
        speeds.append(int(cue['detail']))
    axes[1].plot(range(6),errors,'o-',label='Default' if mode=='vector-free-dock' else mode.capitalize())
    print(mode,'turn errors and RMS',np.round(errors,2),np.sqrt(np.mean(np.square(errors))))
axes[1].axhspan(-5,5,color='green',alpha=.08)
axes[1].set_xticks(range(6),speeds)
axes[1].set(xlabel='Signed right wheel target (mm/s)',ylabel='Steady yaw error (%)')
axes[1].legend(fontsize=8)
fig.savefig(here/'roller-transfer.png',dpi=165)

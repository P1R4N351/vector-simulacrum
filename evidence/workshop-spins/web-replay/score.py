import csv
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
here=Path(__file__).parent
def read(path):
    with path.open() as f:return list(csv.DictReader(f))
def arr(rows,key):
    return np.array([float(r[key]) for r in rows])
def yaw(head,gx,gz):
    theta=head+np.deg2rad(-.66)
    return gx*np.sin(theta)+gz*np.cos(theta)
browser=read(here/'browser.csv');scores=[]
fig,axes=plt.subplots(2,3,figsize=(12,6),layout='constrained')
for sub in ['.','speed-holdout','interleaved']:
    folder=here.parent/sub
    obs=read(folder/'states.csv');events=read(folder/'events.csv');old=read(folder/'predictions.csv')
    for i,(cue,stop) in enumerate(zip([r for r in events if r['event']=='spin'],[r for r in events if r['event']=='stop']),1):
        start,end=float(cue['time_s']),float(stop['time_s']);duration=end-start
        speed=int(cue['detail'])*(100 if sub=='.' else 1)
        sel=[r for r in obs if start-.18<=float(r['time_s'])<end+.36]
        t=arr(sel,'time_s')-start;real=yaw(arr(sel,'head_rad'),arr(sel,'gyro_x'),arr(sel,'gyro_z'))
        m=(t>=duration/2)&(t<duration);br=(t>=duration)&(t<duration+.30)
        ax=axes[int(speed<0),[60,100,150].index(abs(speed))]
        if sub=='interleaved':
            ax.plot(t,real,'.-',label='Measured',c='black',ms=3)
            ax.axvline(duration,ls=':',c='gray',lw=1)
            ax.set_title(f'{abs(speed)} mm/s · '+('CW' if speed<0 else 'CCW'))
        for label,color in [('vector-free-dock','#216799'),('vector-contact42-free-dock','#3f8253'),('archived 3.75 mm','#ba6b23')]:
            if label.startswith('archived'):
                rr=[r for r in old if int(r['turn'])==i]
                pred=arr(rr,'yaw_rad_s')
            else:
                rr=[r for r in browser if r['scene']==label and r['session']==sub and int(r['turn'])==i]
                pred=yaw(arr(rr,'head_rad'),arr(rr,'gx'),arr(rr,'gz'))
            target=np.interp(t,arr(rr,'t_s'),pred)
            scores.append(dict(session=sub,turn=i,speed=speed,scene=label,observed=real[m].mean(),predicted=target[m].mean(),
                               error_pct=100*(target[m].mean()/real[m].mean()-1),brake_rmse=np.mean((target[br]-real[br])**2)**.5))
            if sub=='interleaved':
                ax.plot(t,target,label={'vector-free-dock':'Browser default','vector-contact42-free-dock':'Browser 42 mm'}.get(label,label),color=color)
for ax in axes[:,0]:ax.set_ylabel('Chassis yaw (rad/s)')
for ax in axes[1]:ax.set_xlabel('Time from wheel command (s)')
axes[0,0].legend(fontsize=7);fig.savefig(here/'browser-transfer.png',dpi=165)
with (here/'scores.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=list(scores[0]));w.writeheader();w.writerows(scores)
for label in sorted({r['scene'] for r in scores}):
    print(label)
    for speed in [60,100,150]:
        e=[r['error_pct'] for r in scores if r['scene']==label and abs(r['speed'])==speed]
        print(speed,len(e),np.round(e,1),round(np.sqrt(np.mean(np.square(e))),2))

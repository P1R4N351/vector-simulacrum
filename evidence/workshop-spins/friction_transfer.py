"""Fit scalar tread friction to turns 1–2, then replay turns 3–4 once."""
import csv
import math
from pathlib import Path
from types import SimpleNamespace

import matplotlib.pyplot as plt
import numpy as np
from c7_transfer import gyro_axes, simulate
from simulacrum import params

root = Path(__file__).parent
def read(name):
    with (root/name).open() as f:
        return list(csv.DictReader(f))
states = read('states.csv')
events = read('events.csv')
turns = list(zip([r for r in events if r['event']=='spin'],[r for r in events if r['event']=='stop']))
values = params.values(params.load(model='vector_1'))
default = values['tread_friction']
offset = math.radians(values['imu_pitch_offset_deg'])
def run(index, coefficient):
    cue, stop = turns[index]
    t0,t1 = float(cue['time_s']),float(stop['time_s'])
    duration = t1-t0
    direction = int(cue['detail'])
    seg = SimpleNamespace(t_cmd=t0,t_stop=t1,left=-100*direction,right=100*direction)
    before = next(s for s in reversed(states) if float(s['time_s'])<t0)
    sim = simulate(dict(values,tread_friction=coefficient),dict(head=float(before['head_rad']),lift=float(before['lift_mm'])),seg,3.75)
    obs = [s for s in states if t0<=float(s['time_s'])<t1+.6]
    t = np.array([float(s['time_s'])-t0 for s in obs])
    channels = ('yaw_rad_s','left_mm_s','right_mm_s')
    y = np.array([[gyro_axes(float(s['head_rad'])+offset,[float(s[k]) for k in ('gyro_x','gyro_y','gyro_z')])['yaw_rad_s'],float(s['left_mm_s']),float(s['right_mm_s'])] for s in obs])
    ts = [s['t_s'] for s in sim]
    pred = np.column_stack([np.interp(t,ts,[s[k] for s in sim]) for k in channels])
    steady = (t>=duration/2)&(t<duration)
    brake = (t>=duration)&(t<duration+.3)
    err = pred-y
    pitch = np.array([float(r["gyro_y"]) for r in obs])
    model_pitch = np.interp(t,ts,[s["pitch_rad_s"] for s in sim])
    return dict(observed_yaw_sd_rad_s=np.std(y[steady,0]),model_yaw_sd_rad_s=np.std(pred[steady,0]),
                observed_pitch_rms_rad_s=np.sqrt(np.mean(pitch[steady]**2)),model_pitch_rms_rad_s=np.sqrt(np.mean(model_pitch[steady]**2)),turn=index+1,mu=coefficient,steady_yaw_error_pct=100*(pred[steady,0].mean()/y[steady,0].mean()-1),
                steady_wheel_rmse_mm_s=np.sqrt(np.mean(err[steady,1:]**2)),
                braking_yaw_rmse_rad_s=np.sqrt(np.mean(err[brake,0]**2))), (t,y,pred,duration)
def write(name, rows):
    with (root/name).open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
grid=[]
for mu in sorted(set([default]+[round(.3+.05*j,2) for j in range(25)])):
    a,b=[run(i,mu)[0]['steady_yaw_error_pct'] for i in [0,1]]
    grid.append(dict(mu=mu,fit_rms_pct=math.sqrt((a*a+b*b)/2),ccw_error_pct=a,cw_error_pct=b))
best=min(grid,key=lambda row:row['fit_rms_pct'])['mu']
write('friction-grid.csv',grid)
scores=[]
fig,axes=plt.subplots(2,2,figsize=(10,6),sharex='col',layout='constrained')
for col,index in enumerate([2,3]):
    for mu,style in [(default,'--'),(best,'-')]:
        row,(t,y,pred,duration)=run(index,mu)
        scores.append(row)
        for j,ax in enumerate(axes[:,col]):
            data,target = (y[:,0],pred[:,0]) if j==0 else (y[:,2]-y[:,1],pred[:,2]-pred[:,1])
            if style=='--':
                ax.plot(t,data,'k.',label='Workshop SDK')
                ax.axvline(duration,color='gray',lw=.8)
            ax.plot(t,target,style,label=f'μ = {mu:.2f}')
    axes[0,col].set_title(['Held-out clockwise','Held-out counterclockwise'][col])
    axes[1,col].set_xlabel('Seconds after command')
axes[0,0].set_ylabel('Chassis yaw (rad/s)')
axes[1,0].set_ylabel('Right − left wheel speed (mm/s)')
axes[0,0].legend()
fig.savefig(root/'friction-transfer.png',dpi=160)
write('friction-held-out.csv',scores)
print('default, selected',default,best)
print(grid[[r['mu'] for r in grid].index(default)], min(grid,key=lambda r:r['fit_rms_pct']))
for r in scores: print(r)

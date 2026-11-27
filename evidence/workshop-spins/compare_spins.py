"""Use the archived recorded-pose analysis modules on PYTHONPATH."""
import csv,math,sys
from pathlib import Path
from types import SimpleNamespace
import matplotlib.pyplot as plt
import numpy as np
from c7_transfer import simulate,gyro_axes
from simulacrum import params
mixed = "--interleaved" in sys.argv
fresh = mixed or "--fresh" in sys.argv
base=Path(__file__).parent
if fresh:
    base = base / ("interleaved" if mixed else "speed-holdout")
def load(name):
    with (base/name).open() as f:return list(csv.DictReader(f))
events=load('events.csv');states=load('states.csv')
values=params.values(params.load(model='vector_1'))
commands=[r for r in events if r['event']=='spin']
stops=[r for r in events if r['event']=='stop']
assert len(commands)==len(stops)==(6 if mixed else 4)
fig,axes=plt.subplots(2,len(commands),figsize=(3.5*len(commands),6),sharex='col',layout='constrained')
scores=[]; predictions=[]
for i,(cmd,stop) in enumerate(zip(commands,stops)):
    start=float(cmd['time_s']);end=float(stop['time_s']);speed=int(cmd['detail']) * (1 if fresh else 100);sgn=int(np.sign(speed))
    seg=SimpleNamespace(t_cmd=start,t_stop=end,left=-speed,right=speed)
    prior=next(r for r in reversed(states) if float(r['time_s'])<=start)
    sim=simulate(values,{'head':float(prior['head_rad']),'lift':float(prior['lift_mm'])},seg,3.75)
    predictions.extend([dict(turn=i+1,**r) for r in sim])
    rows=[s for s in states if start-.3<=float(s['time_s'])<=end+.7]
    t=np.array([float(s['time_s'])-start for s in rows]);ts=np.array([s['t_s'] for s in sim])
    offset=math.radians(values['imu_pitch_offset_deg'])
    yaw=np.array([gyro_axes(float(r['head_rad'])+offset,[float(r[k]) for k in ('gyro_x','gyro_y','gyro_z')])['yaw_rad_s'] for r in rows])
    assert all(not int(r['flags']) & (16384|8) for r in rows)
    measured=t>=(end-start)/2; measured &= t<=end-start
    steady=(ts>=(end-start)/2)&(ts<=end-start)
    sy=np.array([s['yaw_rad_s'] for s in sim])
    y0=np.mean(yaw[measured]);y1=np.mean(sy[steady])
    metrics=[i+1,sgn,round(end-start,3),sum(measured),round(y0,4),round(y1,4),round((abs(y1/y0)-1)*100,2)]
    axes[0,i].plot(t,yaw,'.-',label='Measured');axes[0,i].plot(ts,sy,'--',label='Frozen 3.75 mm')
    for side,c in [('left','#347ac0'),('right','#c76b32')]:
        y=np.array([float(r[side+'_mm_s']) for r in rows]);ys=np.array([r[side+'_mm_s'] for r in sim])
        metrics.extend([round(y[measured].mean(),2),round(ys[steady].mean(),2)])
        axes[1,i].plot(t,y,'.-',c=c,label=side);axes[1,i].plot(ts,ys,'--',c=c)
    scores.append(metrics)
    for ax in axes[:,i]:ax.axvline(end-start,lw=.8,c='gray')
    axes[0,i].set_title(f'{abs(speed)} mm/s · '+('CCW' if sgn==1 else 'CW'));axes[1,i].set_xlabel('Seconds after command')
axes[0,0].set_ylabel('Chassis-axis yaw (rad/s)');axes[1,0].set_ylabel('Wheel speed (mm/s)')
axes[0,0].legend(fontsize=8);axes[1,0].legend(fontsize=8)
fig.savefig(base/'spin-overlay.png',dpi=160)
with (base/'scores.csv').open('w') as f:
    w=csv.writer(f);w.writerow('turn sign duration_s steady_packets observed_yaw model_yaw magnitude_error_pct observed_left model_left observed_right model_right'.split());w.writerows(scores)
with (base/'predictions.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=list(predictions[0]));w.writeheader();w.writerows(predictions)

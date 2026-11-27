"""Compare filtered gyro in head and chassis axes, with sample-phase sensitivity."""
import csv
import io
import json
from pathlib import Path
import subprocess
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
def metrics(gx,gy,gz,head,sign):
    theta=np.radians(head)
    yaw=gx*np.sin(theta)+gz*np.cos(theta)
    roll=gx*np.cos(theta)-gz*np.sin(theta)
    # Mean rate separates a persistent tilt from fluctuations about the mean.
    return [np.mean(head),np.degrees(np.arctan2(sign*np.mean(gx),sign*np.mean(gz))),
            sign*np.mean(yaw),sign*np.mean(roll),np.std(roll),np.std(gy)]

scores=[]
fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
for filename in ('unit2-repeat-stops.jsonl','unit2-ccw100.jsonl'):
    events=[json.loads(x) for x in (here.parent/'hw-rec-20261119'/filename).read_text().splitlines()]
    states=[r for r in events if r['k']=='s']
    for start in (r for r in events if r.get('name')=='wheels'):
        begin=start['t']
        end=next(r['issued_t'] for r in events if r.get('name')=='stop' and r['t']>begin)
        times=np.array([r['t']-begin for r in states])
        lo,hi=.55,end-begin-.25
        edges=times[np.searchsorted(times,lo)-1:np.searchsorted(times,hi)+1]
        if np.max(np.diff(edges))>.18: continue
        keep=(times>lo)&(times<hi)
        spin=start['label'];sgn=np.sign(start['right'])
        gyro=np.degrees([r['gyro'] for r,k in zip(states,keep) if k]).T
        head=np.degrees([r['head'] for r,k in zip(states,keep) if k])
        measured=metrics(*gyro,head,sgn)
        scores.append([spin,'physical',0,*measured])
        pose=states[np.searchsorted(times,0)-1]
        trial=dict(spin=spin,left=start['left'],right=start['right'],duration=end-begin,
                   head=np.degrees(pose['head']),lift=pose['lift'])
        data=subprocess.check_output(['node',str(here/'commands.mjs'),'--imu'],input=json.dumps([trial]),text=True)
        sim=np.genfromtxt(io.StringIO(data),delimiter=',',names=True,dtype=None,encoding='utf8')
        simulated=[]
        for shift in range(-30,31,5):
            at=times[keep]+shift/1000
            m=metrics(*(np.interp(at,sim['t'],sim[k]) for k in ('gx','gy','gyro','head_deg')),sgn)
            scores.append([spin,'canonical',shift,*m]);simulated.append(m)
            raw=metrics(*(np.interp(at,sim['t'],sim[k]) for k in ('raw_gx','raw_gy','raw_gz','head_deg')),sgn)
            scores.append([spin,'pre_filter',shift,*raw])
        for ax,k in zip(axes,[4,5]):
            x=len(ax.lines)//2
            vals=np.array(simulated)[:,k]
            ax.plot([x,x],[vals.min(),vals.max()],color='#cf662f',lw=4)
            ax.plot(x,measured[k],'o',color='#1b6586')
for ax,title in zip(axes,['Chassis-frame roll','Head pitch axis']):
    ax.set_title(title);ax.set_ylabel('Steady gyro standard deviation (°/s)')
    ax.set_xticks(range(4),['CW 1','CCW 2','CW 2','Prior CCW']);ax.set_ylim(bottom=0)
fig.suptitle('Recorded packets (dots) and simulated sampling-phase ranges (bars)')
fig.savefig(here/'spin-axes.png',dpi=160)
with (here/'spin-axes.csv').open('w') as f:
    w=csv.writer(f);w.writerow('spin source shift_ms encoder_deg axis_inclination_deg yaw_dps aligned_roll_dps roll_sd_dps pitch_sd_dps'.split());w.writerows(scores)

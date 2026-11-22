"""Transfer the earlier 32 ms pad candidate to two recorded cube landings."""
import csv
from itertools import product

import matplotlib.pyplot as plt
import mujoco
import numpy as np
from release import crossing, here, initial_state
from simulacrum.cube import CubeAccelerometer

def trajectory(angle, axis, sign, shelf_ms, gap_mm, substeps):
    model, d, body, v = initial_state(axis, sign*angle)
    dt = model.opt.timestep
    model.opt.timestep /= substeps
    pads = [i for i in range(model.ngeom) if model.geom(i).name.startswith('cube_rubber_')]
    assert len(pads) == 8
    model.geom_solref[pads,0] = shelf_ms/1000
    d.joint('cube_root').qpos[2] += gap_mm/1000
    mujoco.mj_forward(model,d)
    rest = d.site('cube_chip').xmat.reshape(3,3).T @ [0,0,1]
    force, tilt = [], [1.]
    for _ in range(round(.3/dt)):
        for _ in range(substeps):
            mujoco.mj_step(model,d)
        force.append(d.sensor('cube_accel').data.copy()/(v['gravity_mmps2']*.001))
        tilt.append(np.rad2deg(np.arccos(np.clip(d.xmat[body,8],-1,1)))/angle)
    tilt = np.asarray(tilt)
    t = np.arange(len(tilt))*dt
    duration = 1000*(crossing(t,tilt,.1)-crossing(t,tilt,.9))
    return force, rest, dt, duration, v

def score(force, rest, dt, v, tick, frame, observed):
    chip = CubeAccelerometer(v,dt,frame_phase_s=frame*v['cube_accel_frame_period_ms']/1000,
                             tick_phase_s=tick*v['cube_tick_ms']/1000)
    chip.reset(rest,0.)
    for i,g in enumerate(force,1):
        chip.step(i*dt,g)
    slots = np.concatenate([np.frombuffer(p[2:],'<i2').reshape(3,3) for _,p in chip.messages])
    mag = np.linalg.norm(slots,axis=1)/v['cube_accel_counts_per_g']
    pair, both = 0,0
    for offset in range(3):
        candidates = [i for i in range(len(mag)-10) if (i+offset)%3!=2 and max(mag[i:i+2])<.05]
        pair += bool(candidates)
        both += any(abs(max(mag[i+2:i+11])-observed)<=.05*observed for i in candidates)
    return pair, both, max(mag)

if __name__=='__main__':
    events=np.genfromtxt(here/'low-force-events.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
    rows=[]
    for event, gap, shelf_ms, substeps in product(events,(0,8),(8,32),(4,8)):
        name=event['record']
        angle={'S22B':15.76,'R5':31.5}[name]
        scores, durations = [], []
        for axis, sign in product(range(2),(-1,1)):
            force,rest,dt,duration,v=trajectory(angle,axis,sign,shelf_ms,gap,substeps)
            durations.append(duration)
            for tick,frame in product(np.arange(10)/10,(0.,.5)):
                scores.append(score(force,rest,dt,v,tick,frame,event['next_100ms_peak_g']))
        pairs,both,peak=np.array(scores).T
        rows.append([name,gap,shelf_ms,dt*1000/substeps,int(sum(pairs)),int(sum(both)),len(scores)*3,
                     np.median(durations),np.percentile(peak,5),np.median(peak),np.percentile(peak,95)])
    with (here/'pad-landing.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['run','clearance_mm','pad_ms','physics_ms','pair_cases','pair_peak_cases','cases',
                    'tilt_duration_ms','peak_q05_g','peak_median_g','peak_q95_g']);w.writerows(rows)
    fig,axes=plt.subplots(1,2,figsize=(9,4),layout='constrained')
    for ax,event in zip(axes,events):
        subset=[r for r in rows if r[0]==event['record'] and r[3]==.125]
        for i,r in enumerate(subset):
            ax.vlines(i,r[-3],r[-1],color='#267a87',lw=3)
            ax.plot(i,r[-2],'o',color='#267a87')
        ax.axhline(event['next_100ms_peak_g'],ls='--',color='#9c4226',label='Recorded landing peak')
        ax.set(xticks=range(len(subset)),xticklabels=[f'{r[1]} mm\n{r[2]} ms' for r in subset],
               title=event['record'],xlabel='Release clearance and pad time constant',ylabel='Peak packet magnitude (g)')
        ax.legend(fontsize=8)
    fig.savefig(here/'pad-landing.png',dpi=150)

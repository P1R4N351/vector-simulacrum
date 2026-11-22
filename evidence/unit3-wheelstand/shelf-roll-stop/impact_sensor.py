"""Check the packetized specific-force magnitude on the recorded free return."""
import csv
from itertools import product
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from release import here, initial_state
from simulacrum.cube import CubeAccelerometer

def simulated(axis, sign, phase_fraction, frame_fraction, angle=15.76, gap_mm=0.):
    model, data, body, v = initial_state(axis, sign*angle)
    data.qpos[model.joint("cube_root").qposadr.item()+2] += gap_mm/1000
    dt = model.opt.timestep
    chip = CubeAccelerometer(v, dt, frame_phase_s=frame_fraction*v["cube_accel_frame_period_ms"]/1000, tick_phase_s=phase_fraction*v['cube_tick_ms']/1000)
    mujoco.mj_forward(model,data)
    chip.reset(data.site('cube_chip').xmat.reshape(3,3).T @ [0,0,1], 0.)
    for _ in range(round(.3/dt)):
        mujoco.mj_step(model,data)
        chip.step(data.time,data.sensor('cube_accel').data/(v['gravity_mmps2']*.001))
    samples = []
    for t,payload in chip.app.messages:
        xyz = np.frombuffer(payload[2:],'<i2').reshape(3,3)/v['cube_accel_counts_per_g']
        for slot,g in enumerate(np.linalg.norm(xyz,axis=1)):
            samples.append((t+(slot-2)*v['cube_tick_ms']/1000,g))
    return np.array(samples).T

if __name__ == '__main__':
    c=np.genfromtxt(here/'S22B/cube.csv.gz',names=True,delimiter=',')
    t=c['sec']-31.691988+(c['slot']-2)*.010
    g=np.sqrt(sum(c[k]**2 for k in 'xyz'))/8192
    keep=(t>5.18)&(t<5.52)
    t,g=t[keep],g[keep]
    i=np.argmax(g)
    print('Recorded',t[i],g[i],g.min())
    rows=[]
    fig,ax=plt.subplots(figsize=(7,4),layout='constrained')
    for axis in (0,1):
        for sign in (-1,1):
            for phase,frame_phase in product(np.arange(10)/10, np.arange(4)/4):
                ts,gs=simulated(axis,sign,phase,frame_phase)
                j=np.argmax(gs)
                rows.append([axis,sign,phase,ts[j],gs[j],gs.min(),frame_phase])
                ax.plot((ts-ts[j])*1000,gs,color='#5085ad',alpha=.06,label='Simulated releases' if len(rows)==1 else None)
    ax.plot((t-t[i])*1000,g,'o-',color='#8e3c24',ms=3,label='Physical cube')
    ax.set(xlim=(-80,130),xlabel='Milliseconds relative to each peak',ylabel='Specific-force magnitude (g)',title='Accelerometer during the return to the shelf')
    ax.legend();fig.savefig(here/'impact-sensor.png',dpi=150)
    with (here/'impact-sensor.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['edge_axis','edge_sign','tick_phase_fraction','peak_s','peak_g','minimum_g','frame_phase_fraction']);w.writerows(rows)
    print('Model peak range',min(x[4] for x in rows),max(x[4] for x in rows))

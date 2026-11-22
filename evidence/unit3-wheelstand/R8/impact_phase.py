"""Sweep free-running clocks on a fixed roll trajectory, holding noise and gain fixed."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from tread_speed import run, params, CubeAccelerometer, load_rows
from simulacrum.cube import counts_of

HERE = Path(__file__).resolve().parent
DT = .00025


def main(fixture):
    rest = np.genfromtxt(HERE.parents[1]/'unit3-cube-accelerometer/resting.csv',delimiter=',',names=True)
    gain = np.linalg.norm([rest[k+'_counts'].mean() for k in 'xyz'])/8192
    overrides = {'cube_chip_yaw_deg':180, 'cube_accel_scale':[gain]*3}
    forces = []
    def record(model, data):
        forces.append(data.sensor('chip').data.copy()/9.81)
    *_, reference = run(fixture,DT,True,sensor_overrides=overrides,observer=record)
    settings = params.values(params.load(model='vector_2'))
    settings.update(overrides)
    sensor = CubeAccelerometer(settings,DT)
    magnitudes = []
    for force in forces:
        sensed = sensor._sensed(force,noise=True)
        sensor.filter.update(np.tile(sensed,2))
        word = counts_of(sensor.filter.output[:3],sensor.lsb_per_g,sensor.word_shift,sensor.low_bits)
        magnitudes.append(np.linalg.norm(word)/8192)
    mag = np.asarray(magnitudes)
    frame = round(sensor.frame_s/DT)
    tick = round(sensor.tick_s/DT)
    assert tick % frame == 0
    # Integer indices measure elapsed physics steps, with mag[0] at DT.
    slots = np.arange(1, len(mag)//tick-1)
    rows = []
    unload = []
    physical = np.genfromtxt(HERE/'cube.csv.gz',names=True,delimiter=',')
    zero = next(float(r['sec']) for r in load_rows('actions.csv') if r['action']=='roll cube')
    obs = (physical['sec']-zero>=4.5)&(physical['sec']-zero<5)
    observed = np.linalg.norm(np.column_stack([physical[k][obs] for k in 'xyz']),axis=1).max()/8192
    for f in range(frame):
        for k in range(tick):
            sampled = slots*tick+k
            returned = ((slots+2)//3*3*tick+k)*DT
            latest = sampled-(sampled-f)%frame
            mask = (returned>=4.5)&(returned<5)
            peak = mag[latest[mask]-1].max()
            rows.append((f*DT*1000,k*DT*1000,peak))
            # Select the window on the clock available in both recordings.
            pre = (returned>=4.60)&(returned<4.78125)
            samples = mag[latest[pre]-1]
            unload.append((f*DT*1000,k*DT*1000,len(samples),samples.min(),
                           np.sum(samples<.50),np.sum(samples<.75)))
    baseline = max(np.linalg.norm(np.frombuffer(b[2:],'<i2').astype(float).reshape(3,3),axis=1).max()/8192
                   for t,b in reference.messages if 4.5<=t<5)
    assert np.isclose(rows[0][2],baseline)
    assert abs(baseline-5.32117476)<.005, 'Wrong fixture or simulation baseline'
    for f,k in [(1,3),(frame-1,tick-1)]:
        check = CubeAccelerometer(settings,DT,frame_phase_s=f*DT,tick_phase_s=k*DT)
        for step,force in enumerate(forces,1):
            check.step(step*DT,force)
        peak = max(np.linalg.norm(np.frombuffer(b[2:],'<i2').reshape(3,3).astype(float),axis=1).max()/8192
                   for t,b in check.messages if 4.5<=t<5)
        assert np.isclose(rows[f*tick+k][2],peak)
    phys_t = physical['sec']-zero
    phys_g = np.linalg.norm(np.column_stack([physical[k] for k in 'xyz']),axis=1)/8192
    pre = (phys_t>=4.60)&(phys_t<4.78125)
    earlier = HERE.parent/'R5'
    transport = np.genfromtxt(earlier/'cube.csv.gz',names=True,delimiter=',')
    limits = [float(r['sec']) for r in load_rows('actions.csv',earlier) if r['action']=='set cube down']
    inside = (transport['sec']>=limits[0]) & (transport['sec']<=limits[1])
    tg = np.linalg.norm(np.column_stack([transport[k][inside] for k in 'xyz']),axis=1)/8192
    print('R8 full capture min, slots:',phys_g.min(),len(phys_g))
    print('R5 set-down min, slots below .5 and total:',tg.min(),sum(tg<.5),len(tg))
    scores = np.array(unload)
    print('Pre-impact measured slots, minimum, counts below .5/.75 g:',sum(pre),
          phys_g[pre].min(),sum(phys_g[pre]<.5),sum(phys_g[pre]<.75))
    print('Simulation min/max over phases for [slots, minimum g, count<.50, count<.75]:',
          scores[:,2:].min(axis=0),scores[:,2:].max(axis=0))
    with (HERE/'unloading.csv').open('w') as out:
        w=csv.writer(out,lineterminator='\n')
        w.writerow(['frame_phase_ms','tick_phase_ms','slots','minimum_g','below_half_g','below_075_g'])
        w.writerows(unload)
    fig,ax=plt.subplots(figsize=(9,4),layout='constrained')
    for label,ts,gs in [
        ('Physical slots (receipt time)',phys_t,phys_g),
        ('Simulated slots (packet time)',np.repeat([t for t,_ in reference.messages],3),
         np.linalg.norm(np.vstack([np.frombuffer(b[2:],'<i2').reshape(3,3) for _,b in reference.messages]).astype(float),axis=1)/8192)]:
        show=(ts>=4.59)&(ts<4.91)
        ax.scatter(ts[show],gs[show],label=label,s=18,alpha=.8)
    ax.axvspan(4.60,4.78125,alpha=.12,color='gray',label='Pre-impact window')
    ax.axhline(.5,ls=':',color='.4')
    ax.set(xlabel='Seconds from roll command',ylabel='Specific force magnitude (g)',ylim=(0,5.7))
    ax.legend()
    fig.savefig(HERE/'unloading.png',dpi=160)
    peaks = np.asarray(rows)[:,2]
    print('real, nominal, min/median/max, within real+5%', observed,baseline,np.percentile(peaks,[0,50,100]),sum(peaks<=observed*1.05),len(rows))
    with (HERE/'impact-phase.csv').open('w') as out:
        w=csv.writer(out,lineterminator='\n')
        w.writerow(['frame_phase_ms','tick_phase_ms','peak_packet_g'])
        w.writerows(rows)
    fig,ax=plt.subplots(figsize=(8,3.6),layout='constrained')
    im=ax.imshow(peaks.reshape(frame,tick),origin='lower',aspect='auto',extent=(-.125,tick*.25-.125,-.125,frame*.25-.125))
    ax.set(xlabel='App tick phase (ms)',ylabel='Chip frame phase (ms)',title=f'R8 peak at 4.5–5.0 s · physical maximum {observed:.2f} g')
    fig.colorbar(im,ax=ax,label='Peak packet magnitude (g)')
    fig.savefig(HERE/'impact-phase.png',dpi=160)


if __name__=='__main__':
    main(sys.argv[1])

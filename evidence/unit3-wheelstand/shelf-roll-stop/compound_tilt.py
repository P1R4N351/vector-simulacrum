"""Replay the measured two-axis tilt, including the ambiguity in the flat reference."""
import csv
from itertools import product
import matplotlib.pyplot as plt
import numpy as np
from pad_landing import here, trajectory, CubeAccelerometer
from simulacrum import params

data=np.genfromtxt(here/'S22B/cube.csv.gz',names=True,delimiter=',')
slots=np.column_stack([data[k] for k in 'xyz'])/8192
t=data['sec']-31.691988
values=params.values(params.load(model='vector_2'))
scale=np.array(values['cube_accel_scale'])
flat=np.median(slots[(t>-1)&(t<0)],axis=0)/scale
observed=slots[np.flatnonzero((abs(data['sec']-37.020336)<1e-6)&(data['slot']==1))[0]:][:5]
rows=[]
envelopes={"raw":[], "flat-corrected":[]}
for window,lo,hi in [('held',4.40,5.05),('late',5.15,5.23)]:
    held=np.median(slots[(t>lo)&(t<hi)],axis=0)/scale
    for reference in ('raw','flat-corrected'):
        g=held.copy()
        if reference=='flat-corrected': g-=flat-[0,0,1]
        g/=np.linalg.norm(g)
        # Minimal rotation taking measured body-frame up onto world vertical.
        direction=np.cross(g,[0,0,1]); direction/=np.linalg.norm(direction)
        tilt=np.degrees(np.arccos(g[2]))
        for gap,ms in product((0,1,2,3),(8,32)):
            force,rest,dt,duration,v=trajectory(tilt,direction,1,ms,gap,8)
            assert np.allclose(rest,g)
            xmax=0.; signed=[]; errs=[]; peak_matches=0
            for tick,frame in product(np.arange(10)/10,(0.,.5)):
                chip=CubeAccelerometer(v,dt,frame_phase_s=frame*v['cube_accel_frame_period_ms']/1000,
                                       tick_phase_s=tick*v['cube_tick_ms']/1000)
                chip.reset(rest,0.)
                for j,f in enumerate(force,1): chip.step(j*dt,f)
                a=np.vstack([np.frombuffer(p[2:],'<i2').reshape(3,3) for _,p in chip.messages])/8192
                mag=np.linalg.norm(a,axis=1)
                xmax=max(xmax,max(abs(a[:,0])))
                # Each possible starting slot fixes one of three packet alignments.
                for offset in range(3):
                    cand=[i for i in range(len(a)-4) if (i+offset)%3==1 and max(mag[i:i+2])<.05]
                    if not cand: continue
                    envelopes[reference].extend(a[i:i+8,0] for i in cand if i+8<=len(a))
                    hits=[i for i in cand if abs(mag[i+3]-np.linalg.norm(observed[3]))<=.05*np.linalg.norm(observed[3])]
                    peak_matches+=bool(hits)
                    signed.append(max(a[i+3,0] for i in cand))
                    errs.append(min(np.sqrt(np.mean((a[i+2:i+5]-observed[2:5])**2)) for i in cand))
            rows.append([window,reference,*g,tilt,gap,ms,duration,len(signed),peak_matches,
                         max(signed,default=np.nan),xmax,min(errs,default=np.nan)])
with (here/'compound-tilt.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['window','reference','up_x','up_y','up_z','tilt_deg','gap_mm','pad_ms','duration_ms','pairs_of_60',
                'pair_and_peak_of_60','max_signed_x_at_peak_g','max_abs_x_any_slot_g','min_xyz_rmse_g']);w.writerows(rows)
fig,axes=plt.subplots(1,2,figsize=(8.8,3.7),sharey=True,layout='constrained')
for ax,ms in zip(axes,(8,32)):
    for window,ref in product(('held','late'),('raw','flat-corrected')):
        q=[r for r in rows if r[0]==window and r[1]==ref and r[7]==ms]
        ax.plot([r[6] for r in q],[r[-2] for r in q],'.-',label=f'{window}, {ref}')
    ax.axhline(observed[3,0],color='.3',ls='--',label='Recorded peak-slot X')
    ax.set(xticks=range(4),xlabel='Initial clearance (mm)',title=f'{ms} ms pad time constant')
axes[0].set_ylabel('Largest absolute X sample (g)');axes[-1].legend(fontsize=7)
fig.savefig(here/'compound-tilt.png',dpi=150)

fig,ax=plt.subplots(figsize=(6.8,3.8),layout='constrained')
for reference,samples in envelopes.items():
    curves=np.asarray(samples)
    ax.fill_between(range(8),curves.min(axis=0),curves.max(axis=0),alpha=.25,label=reference)
ax.plot(range(5),observed[:,0],'ko-',label='Recorded X')
ax.axvline(1.5,color='.5',ls=':');ax.set(xlabel='Ordered sample from first low-force slot',ylabel='X specific force (g)')
ax.legend();fig.savefig(here/'compound-slots.png',dpi=150)
print('Largest later signed X', {key:np.max(np.array(val)[:,4:]) for key,val in envelopes.items()})

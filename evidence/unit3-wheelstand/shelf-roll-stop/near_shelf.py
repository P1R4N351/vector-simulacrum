"""Score a camera-constrained release using the measured edge and the ordered packet slots."""
import csv
from itertools import product

import matplotlib.pyplot as plt
import numpy as np
from pad_landing import trajectory, here, CubeAccelerometer

def unit(v):
    return v/np.linalg.norm(v)

def angle(a,b):
    return np.degrees(np.arccos(np.clip(unit(a)@unit(b),-1,1)))

c=np.genfromtxt(here/'S22B/cube.csv.gz',delimiter=',',names=True)
xyz=np.stack([c[k] for k in 'xyz'],axis=1)/8192
t=c['sec']-31.691988
flat=np.median(xyz[(t>-1)&(t<0)],axis=0)
held=np.median(xyz[(t>4.4)&(t<5.05)],axis=0)
# Remove the small static horizontal offset before choosing among the four edgetips.
g0=unit(held-flat+np.array([0.,0.,np.linalg.norm(flat)]))
edges=[]
for axis,sign in product(range(2),(-1,1)):
    _,rest,_,_,_=trajectory(15.76,axis,sign,8,0,4)
    edges.append((angle(rest,g0),axis,sign))
err,axis,sign=min(edges)
print('pre-release edge error (deg), axis, sign',sorted(edges))
event=np.genfromtxt(here/'low-force-events.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
event=next(e for e in event if e['record']=='S22B')
i=np.flatnonzero(np.isclose(c['sec'],event['receipt_s'],rtol=0,atol=1e-6)&(c['slot']==event['first_slot'])).item()
observed=xyz[i:i+5]
assert list(c['slot'][i:i+5])==[1,2,0,1,2]
peak=np.linalg.norm(observed[3])
print('measured contiguous slots',observed)
best_error=float("inf")
rows=[]
for gap,ms,sub in product((0,1,2,3),(8,32),(4,8)):
    force,rest,dt,duration,v=trajectory(15.76,axis,sign,ms,gap,sub)
    for tick,frame in product(np.arange(10)/10,(0.,.5)):
        chip=CubeAccelerometer(v,dt,frame_phase_s=frame*v['cube_accel_frame_period_ms']/1000,
                              tick_phase_s=tick*v['cube_tick_ms']/1000)
        chip.reset(rest,0.)
        for j,g in enumerate(force,1): chip.step(j*dt,g)
        slots=np.vstack([np.frombuffer(p[2:],'<i2').reshape(3,3) for _,p in chip.messages])/8192
        mags=np.linalg.norm(slots,axis=1)
        for phase in range(3):
            pairs=[j for j in range(len(slots)-10)
                   if (j+phase)%3==1 and max(mags[j:j+2])<.05]
            ok=[j for j in pairs if abs(mags[j+3]-peak)<=.05*peak]
            # First two slots are near weightlessness. Compare all three impact slots in XYZ.
            minerr=min((np.sqrt(np.mean((slots[j+2:j+5]-observed[2:5])**2)) for j in pairs),default=np.nan)
            if sub==8 and minerr<best_error:
                start=min(pairs,key=lambda j:np.mean((slots[j+2:j+5]-observed[2:5])**2))
                best_error=minerr
                best_slots=slots[start:start+5]
                best_case=(gap,ms)
            direction=min((angle(slots[j+3],observed[3]) for j in ok),default=np.nan)
            rows.append([gap,ms,dt*1000/sub,tick,frame,phase,duration,int(bool(pairs)),
                         int(bool(ok)),int(bool(ok) and 27.6<=duration<=84),minerr,direction,max((abs(slots[j+3,0]) for j in pairs),default=np.nan),
                         min((angle(slots[j+3],observed[3]) for j in pairs if mags[j+3]>.1),default=np.nan)])
a=np.array(rows)
with (here/'near-shelf.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['gap_mm','pad_ms','step_ms','tick_phase','frame_phase','packet_phase','tilt_ms','slot12_low','slot1_peak_match','with_camera','min_impact_xyz_rmse_g','matching_peak_direction_error_deg','max_abs_x_at_peak_slot_g','min_direction_deg_above_point1g'])
    w.writerows(rows)
fig,axes=plt.subplots(1,2,figsize=(8,3.5),sharey=True,layout='constrained')
for ax,ms in zip(axes,(8,32)):
    counts=[]
    for gap in range(4):
        b=a[(a[:,0]==gap)&(a[:,1]==ms)&(a[:,2]==.125)]
        counts.append(b[:,7:10].sum(axis=0))
        print(gap,ms,counts[-1],b[0,6])
    for k,label in enumerate(('Low pair','Pair and timed peak','Also camera timing')):
        ax.plot(range(4),np.array(counts)[:,k],['o-','s--','x:'][k],label=label)
    ax.set(xticks=range(4),xlabel='Initial clearance (mm)',title=f'{ms} ms pad contact')
axes[0].set_ylabel('Matching cases out of 60');axes[-1].legend(fontsize=8)
fig.savefig(here/'near-shelf.png',dpi=150)

fig,axes=plt.subplots(1,3,figsize=(9,3.3),sharey=True,layout='constrained')
for k,ax in enumerate(axes):
    ax.plot(observed[:,k],'o-',label='Recorded')
    ax.plot(best_slots[:,k],'s--',label='Closest simulated sequence')
    ax.axvline(1.5,color='.6',lw=1)
    ax.set(xticks=range(5),xticklabels=['1','2','0','1','2'],xlabel='Slot in received packet',title='XYZ'[k])
axes[0].set_ylabel('Specific force (g)');axes[-1].legend(fontsize=7)
fig.suptitle(f'Shelf landing · closest three-axis replay ({best_case[0]} mm clearance, {best_case[1]} ms pads)')
fig.savefig(here/'near-shelf-axes.png',dpi=150)

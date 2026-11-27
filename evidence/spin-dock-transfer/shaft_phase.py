"""Test whether cumulative shaft position predicts a repeatable component of roll."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi

here=Path(__file__).resolve().parent
def read(p):
    return np.genfromtxt(p,delimiter=',',names=True,dtype=None,encoding='utf8')
h=read(here.parent/'hw-rec-20261119/unit2-native-stops.csv')
s=read(here/'native-axes-trace.csv')
count_mm=.96*29*.25*np.pi/172.3
train=('ccw_1','cw_1')
test=('ccw_2','cw_2')
clock=np.arange(-540,-110,5)
keep=(clock>=-440)&(clock< -120)
filt=butter(2,20,fs=200,output='sos')
def window(x):
    low=sosfilt(filt,x,zi=sosfilt_zi(filt)*x[0])[0][keep]
    return low-low.mean()
def roll(gx,gz,theta):
    return gx*np.cos(theta)-gz*np.sin(theta)

obs={};base={};wheels={}
for spin in train+test:
    r=h[h['turn']==spin];t=r['ms_from_last_edge']
    sim=s[(s['turn']==spin)&(s['variant']=='canonical')]
    obs[spin]=window(np.interp(clock,t,np.degrees(roll(r['head_gx'],r['head_gz'],r['head_rad']))))
    base[spin]=window(np.interp(clock,sim['ms'],roll(sim['gx'],sim['gyro_dps'],np.radians(sim['head_deg']))))
    wheels[spin]=[(np.interp(clock,t,sign*r[k]*count_mm),np.interp(clock,t,r[v]))
                  for k,v,sign in [('left_count','left_mmps',1),('right_count','right_mmps',-1)]]

def spatial(spin,wavelength):
    columns=[]
    for position,velocity in wheels[spin]:
        phase=position*2*np.pi/wavelength
        # For an angle ripple periodic in travel, its time derivative changes sign on reverse.
        columns.extend(window(velocity*fn(phase)) for fn in (np.cos,np.sin))
    return np.column_stack(columns)

def fit(names,wavelength):
    x=np.vstack([spatial(n,wavelength) for n in names])
    y=np.concatenate([obs[n]-base[n] for n in names])
    # Nearby left and right phases are highly correlated during balanced spins.
    # Regularize using a fixed fraction of total feature power.
    return np.linalg.solve(x.T@x+np.eye(4)*.01*np.trace(x.T@x)/4,x.T@y)
def mse(n,w,coef):
    err=obs[n]-base[n]-spatial(n,w)@coef
    return np.mean(err**2)
lengths=np.arange(5,40.01,.25)
# Absolute counts and zero offsets stay fixed between the opposite-direction trials.
errors=[sum(mse(n,w,fit([other],w)) for n,other in (train,train[::-1])) for w in lengths]
w=lengths[np.argmin(errors)]
coef=fit(train,w)
scores=[]
fig,axes=plt.subplots(2,2,figsize=(10,5.5),layout='constrained',sharex=True,sharey=True)
for n,ax in zip(train+test,axes.flat):
    predicted=base[n]+spatial(n,w)@coef
    scores.append((n,w,np.sqrt(np.mean((obs[n]-base[n])**2)),np.sqrt(mse(n,w,coef))))
    ax.plot(clock[keep],obs[n],label='Physical',color='#222222')
    ax.plot(clock[keep],base[n],label='Canonical',color='#be6036')
    ax.plot(clock[keep],predicted,label='Shaft-phase prediction',color='#287582')
    ax.set_title(n.upper()+(' (development)' if n in train else ' (held out)'))
for ax in axes[-1]: ax.set_xlabel('Milliseconds from final encoder edge')
for ax in axes[:,0]: ax.set_ylabel('Centered roll (°/s)')
axes[0,0].legend(fontsize=8)
fig.savefig(here/'shaft-phase.png',dpi=160)
with (here/'shaft-phase.csv').open('w') as f:
    out=csv.writer(f);out.writerow(['spin','wavelength_mm','baseline_rmse','phase_rmse']);out.writerows(scores)
print(scores)

# Controls chosen from nominal axle dimensions, independently of any gyro fit.
with (here/'shaft-geometry-controls.csv').open('w') as f:
    out=csv.writer(f);out.writerow(['reference','wavelength_mm','spin','scored_travel_mm','baseline_rmse','phase_rmse'])
    for label,period in [('front revolution',29*np.pi),('rear revolution',21.5*np.pi),
                         ('front half revolution',29*np.pi/2),('rear half revolution',21.5*np.pi/2)]:
        beta=fit(train,period)
        for n in test:
            travel=np.mean([abs(pos[keep][-1]-pos[keep][0]) for pos,_ in wheels[n]])
            out.writerow([label,period,n,travel,np.sqrt(np.mean((obs[n]-base[n])**2)),np.sqrt(mse(n,period,beta))])

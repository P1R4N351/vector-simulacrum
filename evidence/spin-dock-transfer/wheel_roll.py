"""Test whether fluctuations in tread speed transfer to held-out steady roll."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi

here=Path(__file__).resolve().parent
def read(path):
    return np.genfromtxt(path,names=True,delimiter=',',dtype=None,encoding='utf8')

hw=read(here.parent/'hw-rec-20261119/unit2-native-stops.csv')
sim=read(here/'native-axes-trace.csv')
grid=np.arange(-540,-110,5)
use=(grid>=-440)&(grid< -120)
train=('ccw_1','cw_1')
test=('ccw_2','cw_2')
low=butter(2,20,fs=200,output='sos')
def sample(t,signal):
    x=np.interp(grid,t,signal)
    return sosfilt(low,x,zi=sosfilt_zi(low)*x[0])[0]

def center(x):
    return x-x.mean(axis=0)

def roll(x,z,theta):
    return x*np.cos(theta)-z*np.sin(theta)

traces={}
for name in train+test:
    h=hw[hw['turn']==name]
    s=sim[(sim['turn']==name)&(sim['variant']=='canonical')]
    assert h['ms_from_last_edge'][0]<=grid[0] and h['ms_from_last_edge'][-1]>=grid[-1]
    hroll=np.degrees(roll(h['head_gx'],h['head_gz'],h['head_rad']))
    sroll=roll(s['gx'],s['gyro_dps'],np.radians(s['head_deg']))
    measured=center(sample(h['ms_from_last_edge'],hroll)[use])
    base=center(sample(s['ms'],sroll)[use])
    traces[name]=(measured,base,{label:np.array([sample(t,left),sample(t,right)]).T
        for label,t,left,right in [('physical',h['ms_from_last_edge'],h['left_mmps'],h['right_mmps']),
                                    ('simulated',s['ms'],s['left_mmps'],s['right_mmps'])]})

def features(name,source,delay):
    # Signed wheel fluctuations share one coefficient per physical side across directions.
    at=np.flatnonzero(use)-delay//5
    return center(traces[name][2][source][at])

def rms(x):
    return np.sqrt(np.mean(x*x))

residual=np.concatenate([traces[n][0]-traces[n][1] for n in train])
options=[]
for lag in range(0,95,5):
    x=np.concatenate([features(n,'physical',lag) for n in train])
    coef=np.linalg.lstsq(x,residual,rcond=None)[0]
    options.append((rms(residual-x@coef),lag,coef))
_,delay,weights=min(options,key=lambda row:row[0])
fig,axes=plt.subplots(2,2,figsize=(10,5.5),layout='constrained',sharex=True,sharey=True)
scores=[]
for name,ax in zip(train+test,axes.flat):
    target,base,_=traces[name]
    ax.plot(grid[use],target,label='Measured roll',color='#222222')
    ax.plot(grid[use],base,label='Canonical',color='#b15429')
    for source in ('physical','simulated'):
        pred=base+features(name,source,delay)@weights
        ax.plot(grid[use],pred,label=source+' wheel input',lw=1)
        scores.append([name,source,delay,*weights,rms(target-base),rms(target-pred)])
    ax.set_title(name.upper()+(' (fit)' if name in train else ' (held out)'))
axes[0,0].legend(fontsize=7)
for ax in axes[-1]: ax.set_xlabel('Milliseconds from final physical wheel edge')
for ax in axes[:,0]: ax.set_ylabel('Centered roll rate (°/s)')
fig.savefig(here/'wheel-roll.png',dpi=160)
with (here/'wheel-roll.csv').open('w') as f:
    w=csv.writer(f);w.writerow('spin wheel_source lag_ms left_coef right_coef baseline_rmse_dps candidate_rmse_dps'.split());w.writerows(scores)

# Preserve the smoothness and amplitude of each correction, scrambling its timing only.
for source in ('physical','simulated'):
    errors=[]
    for n in test:
        measured,baseline,_=traces[n]
        correction=features(n,source,delay)@weights
        errors.append(np.array([np.mean((measured-baseline-np.roll(correction,k))**2)
                                for k in range(len(correction))]))
    pairs=np.sqrt((errors[0][:,None]+errors[1][None,:])/2)
    print(source,'unshifted',pairs[0,0],'median control',np.median(pairs),
          'fraction of shifted alignments as good',np.mean(pairs<=pairs[0,0]))

"""Compare robot-clock transverse gyro power in a 400 ms steady-turn window."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import periodogram, butter, sosfiltfilt

here=Path(__file__).resolve().parent
def read(path):
    return np.genfromtxt(path,names=True,delimiter=',',dtype=None,encoding='utf8')

hw=read(here.parent/'hw-rec-20261119/unit2-native-stops.csv')
sim=read(here/'native-axes-trace.csv')
solver=read(here/'native-solver-trace.csv')
grid=np.arange(-520,-120,5)
spins=('ccw_1','cw_1','ccw_2','cw_2')
fig,axes=plt.subplots(2,4,figsize=(13,6),layout='constrained',sharex=True,sharey='row')
scores=[]
for col,name in enumerate(spins):
    r=hw[hw['turn']==name]
    t=r['ms_from_last_edge']
    inside=(t>=grid[0]-15)&(t<=grid[-1]+15)
    assert max(np.diff(t[inside]))<=11 and min(abs(r['left_mmps'][inside]))>=80
    signals=[('Physical',t,np.degrees(r['head_gx']),np.degrees(r['head_gy']),np.degrees(r['head_gz']),r['head_rad'])]
    for variant in ('canonical','shaft_count_position'):
        s=sim[(sim['turn']==name)&(sim['variant']==variant)]
        signals.append((variant,s['ms'],s['gx'],s['gy'],s['gyro_dps'],np.radians(s['head_deg'])))
        if variant=='canonical':
            q=solver[solver['spin']==name]
            assert np.allclose(np.diff(q['ms']),1)
            lowpass=butter(10,85,fs=1000,output="sos")
            raw=[sosfiltfilt(lowpass,q[k])[::5] for k in ('raw_gx','raw_gy','raw_gz','head_rad')]
            signals.append(('pre-filter (anti-aliased)',q['ms'][::5],*raw))
    for source,clock,gx,gy,gz,theta in signals:
        roll=gx*np.cos(theta)-gz*np.sin(theta)
        for ax,signal,axis in zip(axes[:,col],(roll,gy),('roll','pitch')):
            x=np.interp(grid,clock,signal)
            assert np.all(np.isfinite(x))
            f,p=periodogram(x,200,window='hann',detrend='constant',scaling='density')
            bands=[np.sqrt(np.sum(p[(f>=low)&(f<high)])*(f[1]-f[0])) for low,high in ((2.5,20),(20,45),(45,90))]
            scores.append([name,source,axis,np.std(x),*bands])
            ax.semilogy(f[1:],np.maximum(p[1:],1e-7),lw=1,alpha=.8,label=source.replace('_',' '))
    axes[0,col].set_title(name.upper())
    axes[1,col].set_xlabel('Frequency (Hz)')
axes[0,0].legend(fontsize=7)
axes[0,0].set_ylabel('Roll PSD ((°/s)²/Hz)')
axes[1,0].set_ylabel('Pitch PSD ((°/s)²/Hz)')
fig.savefig(here/'native-axes.png',dpi=160)
with (here/'native-axes.csv').open('w') as f:
    w=csv.writer(f);w.writerow('spin source axis sd_dps rms_2p5_to20 rms_20to45 rms_45to90'.split());w.writerows(scores)

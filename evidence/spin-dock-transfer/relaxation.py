"""Select contact relaxation on low-band roll, then score yaw and roll transfer."""
import csv
import io
import os
from pathlib import Path
import subprocess
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import periodogram

here=Path(__file__).resolve().parent
def read(source):
    return np.genfromtxt(source,names=True,dtype=None,encoding='utf8',delimiter=',')

hw=read(here.parent/'hw-rec-20261119/unit2-native-stops.csv')
grid=np.arange(-520,-120,5)
def metrics(t,gx,gz,theta):
    roll=np.interp(grid,t,gx*np.cos(theta)-gz*np.sin(theta))
    yaw=np.interp(grid,t,gx*np.sin(theta)+gz*np.cos(theta))
    f,p=periodogram(roll,fs=200,window='hann')
    return np.sqrt(2.5*p[(f>=2.5)&(f<20)].sum()),abs(yaw.mean())

spins=['ccw_1','cw_1','ccw_2','cw_2']
observed={}
for spin in spins:
    h=hw[hw['turn']==spin]
    observed[spin]=metrics(h['ms_from_last_edge'],np.degrees(h['head_gx']),np.degrees(h['head_gz']),h['head_rad'])
results=[]
def replay(ms,step):
    output=subprocess.check_output(['node',str(here/'replay.mjs'),'--axes'],text=True,
                                   env={**os.environ,'VECTOR_SPIN_RELAX_MS':str(ms),'VECTOR_SPIN_STEP_MS':str(step)})
    sim=read(io.StringIO(output))
    rows=[]
    for spin in spins:
        r=sim[sim['turn']==spin]
        roll,yaw=metrics(r['ms'],r['gx'],r['gyro_dps'],np.radians(r['head_deg']))
        hr,hy=observed[spin]
        rows.append((ms,spin,hr,roll,hy,yaw))
    return rows
for ms in (2,4,6,8,12,16):
    results.extend(replay(ms,1))
def training_loss(ms):
    return np.mean([np.log(roll/hr)**2 for m,n,hr,roll,hy,yaw in results if m==ms and n.endswith('_1')])
chosen=min((2,4,6,8,12,16),key=training_loss)
print('Selected relaxation (ms):',chosen)
with (here/'relaxation.csv').open('w') as f:
    w=csv.writer(f);w.writerow('ms spin measured_roll_dps sim_roll_dps measured_yaw_dps sim_yaw_dps'.split());w.writerows(results)
fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
for spin in spins:
    rows=[r for r in results if r[1]==spin]
    for ax,i in zip(axes,(3,5)):
        ax.plot([r[0] for r in rows],[100*(r[i]/r[i-1]-1) for r in rows],
                marker='o',ls='--' if spin.endswith('_1') else '-',label=spin)
for ax in axes:
    ax.axhline(0,color='.5',lw=1);ax.axvline(chosen,color='.6',ls=':');ax.set_xlabel('Tread contact relaxation (ms)')
axes[0].set_ylabel('Low-band roll RMS error (%)');axes[1].set_ylabel('Steady yaw rate error (%)')
axes[0].legend(fontsize=8)
fig.savefig(here/'relaxation.png',dpi=165)

with (here/'relaxation-step.csv').open('w') as f:
    w=csv.writer(f);w.writerow('step_ms contact_ms spin measured_roll_dps sim_roll_dps measured_yaw_dps sim_yaw_dps'.split())
    for step in (.5,.25):
        for ms in (4,chosen):
            w.writerows((step,*row) for row in replay(ms,step))

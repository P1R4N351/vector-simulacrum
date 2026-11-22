"""Compare the loaded encounters using the last state before rapid lift descent."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
fig,axes=plt.subplots(4,1,figsize=(9,8),sharex=True,layout='constrained')
with (here/'comparison.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['trial','anchor_from_command_s','peak_pitch_deg','peak_cube_g','cube_counter_increments','cube_samples_from_cmd','last_pitch_deg'])
    for name in ('W4','W8','W12'):
        p=here.parent/name
        zero=next(float(r['sec']) for r in csv.DictReader((p/'actions.csv').open()) if r['action']=='pop_a_wheelie')
        s=np.genfromtxt(p/'state.csv.gz',names=True,delimiter=',');s=s[s['sec']>=zero]
        c=np.genfromtxt(p/'cube.csv.gz',names=True,delimiter=',');c=c[c['sec']>=zero]
        anchor=s['sec'][np.flatnonzero((np.diff(s['lift']) < -10)&(s['lift'][:-1]>80))[0]]
        force=np.linalg.norm([c[k] for k in 'xyz'],axis=0)/8192
        pitch=np.degrees(s['pitch']);t=s['sec']-anchor
        w.writerow([name,anchor-zero,max(pitch),max(force),int((c['tap'][-1]-c['tap'][0])%256),len(c),pitch[-1]])
        for ax,series in zip(axes[:3],(s['lift'],pitch,(s['left']+s['right'])/2)):
            ax.plot(t,series,'.-',label=name,ms=3)
        axes[3].plot(c['sec']-anchor,force,'.',ms=3,label=name)
for ax,label in zip(axes,('Lift height (mm)','Chassis pitch (°)','Mean wheel speed (mm/s)','Cube specific force (g)')):
    ax.set(ylabel=label,xlim=(-.30,.8));ax.axvline(0,color='.6',lw=.7)
axes[0].legend();axes[-1].set_xlabel('Seconds from last sample before rapid lift descent')
fig.savefig(here/'comparison.png',dpi=160)

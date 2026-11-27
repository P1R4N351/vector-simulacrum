"""Plot edge release outcomes with an independent rigid-pivot energy check."""
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
r=np.genfromtxt(here/'edge-release.csv',names=True,delimiter=',')
angles=np.unique(r['angle_deg']);steps=np.unique(r['step_ms'])
assert len(r)==7*19*2 and np.all(np.isfinite(r['final_deg']))
fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
for step in steps:
    critical=[];dur=[]
    for a in angles:
        s=r[(r['step_ms']==step)&(r['angle_deg']==a)]
        won=np.isfinite(s['settled_new_face_s'])
        assert np.all(np.diff(won.astype(int))>=0)
        critical.append(s['rate_dps'][won].min() if won.any() else np.nan)
        if a==41.3:
            axes[1].plot(s['rate_dps'][won],s['settled_new_face_s'][won],'.-',label=f'{step:g} ms step')
    axes[0].plot(angles,critical,'o-',label=f'{step:g} ms step')
h=.0184857955+.00416761364
a=np.linspace(30,50)
rise=np.where(a<45,h*(np.sqrt(2)-np.sin(np.radians(a))-np.cos(np.radians(a))),0)
omega=np.sqrt(2*.05*9.81*rise/(1.6875e-5+2*.05*h*h))
axes[0].plot(a,np.degrees(omega),'k--',label='Rigid pivot, no loss')
axes[0].set(xlabel='Initial cube tilt (°)',ylabel='Smallest tested forward rate to settle on +Y (°/s)')
axes[1].axhspan(.27563,.39663,alpha=.15,color='orange',label='Recorded optical interval')
axes[1].set(xlabel='Initial forward rate at 41.3° tilt (°/s)',ylabel='Time to stay within 2° of +Y (s)')
for ax in axes:ax.legend(fontsize=8)
fig.savefig(here/'edge-release.png',dpi=165)

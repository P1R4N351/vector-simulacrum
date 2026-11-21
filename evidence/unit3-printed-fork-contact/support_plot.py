from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
here=Path(__file__).resolve().parent
samples=np.genfromtxt(here/'support.csv',delimiter=',',names=True)
r=samples[samples['trial']==1]
fig,(top,bottom)=plt.subplots(2,1,sharex=True,figsize=(7.6,4.4))
top.plot(r['sec'],r['lift_mm'],color='#30446b')
top.set_ylabel('Wrist height (mm)')
bottom.plot(r['sec'],r['up_N'],color='#007f77',lw=1)
bottom.axhline(.050*9.81,ls='--',lw=1,color='0.35',label='Assumed 50 g cube weight')
bottom.set(xlim=(4.35,6.8),xlabel='Seconds from approach command',ylabel='Upward fork force (N)')
bottom.legend(frameon=False);top.set_title('Trial 1 · contact force in the printed-fork replay')
for ax in (top,bottom):ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(here/'fork-force.png',dpi=160)

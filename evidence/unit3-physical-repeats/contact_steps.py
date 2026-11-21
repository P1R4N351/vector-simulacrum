"""Compare the same shelf-contact replay over physics integration steps."""
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

out=Path(__file__).resolve().parent/'E'
fig,axes=plt.subplots(1,3,figsize=(9,3.6),layout='constrained')
for ax,key,title,target in zip(axes,['yaw_change_deg','packet_peak_g','packet_taps_in_window'],
                              ['Yaw change (°)','Peak specific force (g)','Tap candidates'],
                              [(14.148,18.013),(1.078,1.078),(1,1)]):
    for index,suffix in enumerate(['','-sub2','-sub4','-sub8-nominal','-sub16-nominal']):
        rows=np.genfromtxt(out/f'printed-contact-cadence{suffix}-replay.csv',names=True,delimiter=',',encoding='utf8',dtype=None)
        rows=np.atleast_1d(rows)
        nominal=(rows['cube_dx_mm']==0)&(rows['cube_dy_mm']==0)
        ax.scatter(np.repeat(index,len(rows)-1),rows[key][~nominal],color='.65',s=12)
        ax.scatter(index,rows[key][nominal],color='#124c82',s=45)
    if target[0]==target[1]:ax.axhline(target[0],color='#258048',ls='--')
    else:ax.axhspan(*target,color='#c3e6cd')
    ax.set(title=title,xticks=range(5),xticklabels=['1','.5','.25','.125','.0625'],xlabel='Physics step (ms)')
fig.suptitle('Recorded trajectory · blue: SDK cube pose · gray: offset positions · green: physical')
fig.savefig(out/'contact-steps.png',dpi=150)

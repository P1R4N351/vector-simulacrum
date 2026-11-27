from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
with (here/'scores.csv').open() as f:
    data=list(csv.DictReader(f))
fig,axes=plt.subplots(1,3,figsize=(11,4),layout='constrained')
for ax,(session,title) in zip(axes,[('recovery','Selection · first unit'),('target-crossover','Held out · first unit'),('target-transfer','Held out · second unit')]):
    sub=[r for r in data if r['session']==session]
    peaks=sorted({int(r['peak_mNm']) for r in sub})
    for up,label,offset in [(True,'Raises',-.18),(False,'Lowers',.18)]:
        y=[np.mean([float(r['rmse_mm']) for r in sub if int(r['peak_mNm'])==p and (r['target_mm']=='64')==up]) for p in peaks]
        ax.bar(np.arange(len(peaks))+offset,y,width=.35,label=label)
    ax.set_xticks(np.arange(len(peaks)),[str(p) if p else 'Base' for p in peaks])
    ax.set(title=title,xlabel='Peak friction torque (mN·m)',ylim=(0,4))
    ax.set_ylim(0,max(4,ax.dataLim.ymax*1.1))
axes[0].set_ylabel('Mean stroke RMSE (mm)')
axes[0].legend()
fig.savefig(here/'friction.png',dpi=160)

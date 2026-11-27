from pathlib import Path
import csv
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
with (here/'scores.csv').open() as f:
    rows=list(csv.DictReader(f))
fig,axes=plt.subplots(1,2,figsize=(9,3.5),layout='constrained')
for ax,joint in zip(axes,('head','lift')):
    sub=[r for r in rows if r['joint']==joint]
    for r in sub:
        ax.plot([0,1],[float(r['old_median_abs_deg']),float(r['new_median_abs_deg'])],marker='o',label=r['session'])
    ax.set(yscale='log',xticks=[0,1],xticklabels=['Previous','Calibrated'],title=joint.capitalize(),ylabel='Median distance to encoder grid (degrees)')
axes[0].legend(fontsize=8)
fig.savefig(here/'encoder-grid.png',dpi=160)

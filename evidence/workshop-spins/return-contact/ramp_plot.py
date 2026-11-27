import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent
fig, axes = plt.subplots(1, 2, figsize=(10, 4), layout='constrained')
for mode, label in [('keep','Default rear tips'),('clear','Rear tips clear of dock')]:
    with (root/f'load-{mode}-1.csv').open() as src:
        rows=list(csv.DictReader(src))
    times={}
    for r in rows:
        times[float(r['t'])]=float(r['pitch_deg'])
    t=np.array(list(times))
    axes[0].plot(t-t[0], list(times.values()),'.',ms=2,label=label)
axes[0].legend(fontsize=8)
axes[0].set(xlabel='Time since first sampled backup (s)',ylabel='Pitch (degrees)',title='Autonomous return at −30 mm/s')
with (root/'bench.csv').open() as src:
    bench=list(csv.DictReader(src))
for i, (mode,label) in enumerate([('default','Default'),('clear','Tips cleared'),('axles','Tips cleared, 48 mm axles')]):
    rr=[r for r in bench if r['variant']==mode]
    for j,r in enumerate(rr):
        axes[1].plot(j+(i-1)*.18,float(r['max_depth_mm']),'o' if r['contact']=='1' else 'x',color=['#365783','#b86923','#338559'][i],label=label if j==0 else None)
axes[1].legend(fontsize=8)
axes[1].set_xticks(range(5),['0 / 0','−3 / −1','+3 / +1','−3 / +1','+3 / −1'])
axes[1].set(xlabel='Starting lateral offset (mm) / heading (°)',ylabel='Farthest robot origin past lip (mm)',title='Straight backup · circle = charger flag')
fig.savefig(root/'ramp-load.png',dpi=160)

"""Score the microsecond-rounded S22B calibration on untouched intervals."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent
def read(name):
    with (root/name).open() as f:
        return list(csv.DictReader(f))

phases = read('supervisor-phase.csv')
loop_ms = round(float(phases[0]['S22B_period_ms']) / 6 * 1000) / 1000
awake = [int(r['payload_ms'])-int(r['anchor_ms']) for r in phases if r['run']!='S22B']
docked = [float(r['payload_interval_ms']) for r in read('sdk-axis-clock.csv') if r['fit_burst']=='False']
fig,axes = plt.subplots(1,2,figsize=(8,3.5),layout='constrained')
for ax,label,periods,loops in zip(axes,('Awake','Docked S'),(awake,docked),(6,50)):
    for loop in (5.080,loop_ms):
        step = loops*loop
        errors = abs(periods - np.rint(np.array(periods)/step)*step)
        ax.plot(np.arange(1,len(errors)+1),np.sort(errors),'.-',label=f'{loop:.3f} ms loop')
        print(label,loop,len(errors),np.median(errors),errors.max())
    ax.set(title=label,xlabel='Intervals ranked by error',ylabel='Absolute timestamp grid error (ms)')
    ax.legend(fontsize=8)
fig.savefig(root/'supervisor-transfer.png',dpi=165)

"""Audit contiguous low-force samples in the archived shelf cube recordings."""
from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt

here = Path(__file__).resolve().parent
wheel = here.parent
files = sorted(wheel.glob('*/cube.csv.gz')) + sorted(here.glob('*/cube.csv.gz'))
files += sorted((wheel.parent/'unit3-physical-repeats').glob('*/cube.csv.gz'))
files += sorted(wheel.glob('stationary*.csv.gz'))
summary, events = [], []
fig, ax = plt.subplots(figsize=(8,4),layout='constrained')
for path in files:
    data = np.genfromtxt(path, names=True, delimiter=',')
    if not len(data): continue
    old = 'x_counts' in data.dtype.names
    t = data['receipt_s' if old else 'sec']
    xyz = np.column_stack([data[k+'_counts' if old else k] for k in 'xyz'])
    g = np.linalg.norm(xyz,axis=1)/8192
    name = path.parent.name if path.name=='cube.csv.gz' else path.name.split('.')[0]
    if old: name = 'repeat-'+name
    low = g < .05
    # Same packet only, so transport jitter cannot turn a gap into a pair.
    pair = np.flatnonzero(low[:-1] & low[1:] & (data['slot'][1:]==data['slot'][:-1]+1) & (t[1:]==t[:-1]))
    summary.append([name,len(t),round(g.min(),5),int(low.sum()),len(pair)])
    for i in pair:
        j = np.arange(max(0,i-6),min(i+13,len(t)))
        rel = t[j]-t[i]+.01*(data['slot'][j]-data['slot'][i])
        after = (j>i+1)&(rel>=0)&(rel<=.1)
        peak = g[j][after].max() if after.any() else np.nan
        events.append([name,t[i],int(data['slot'][i]),g[i],g[i+1],peak])
        ax.plot(rel*1000,g[j],'.-',label=f'{name} at {t[i]:.2f} s')
ax.set(xlabel='Milliseconds from first low-force sample (receipt and slot estimate)',ylabel='Specific-force magnitude (g)',title='Consecutive samples below 0.05 g in one cube packet')
if events: ax.legend(fontsize=7)
fig.savefig(here/'low-force.png',dpi=150)
for filename,cols,rows in [('low-force-summary.csv',['record','slots','min_g','below_005g','adjacent_pairs_in_packet'],summary),('low-force-events.csv',['record','receipt_s','first_slot','first_g','second_g','next_100ms_peak_g'],events)]:
    with (here/filename).open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(cols);w.writerows(rows)

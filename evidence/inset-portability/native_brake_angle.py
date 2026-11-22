"""Score frozen wheel-input reconstructions at physical wheel-speed crossings."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
data = here.parent / 'hw-rec-20261119'
def read(name):
    with (data/name).open() as handle:
        return list(csv.DictReader(handle))

raw = read('unit2-native-stops.csv')
replay = read('hybrid-encoder-replay.csv')
methods = ['reported', 'count_only_end_time', 'hybrid_end_time']
fig, axes = plt.subplots(2, 2, figsize=(8, 5.8), layout='constrained')
with (here/'native-brake-angle.csv').open('w',newline='') as handle:
    writer = csv.writer(handle)
    writer.writerow(['spin','crossing_fraction','method','measured_deg','predicted_deg','error_deg','error_pct'])
    for label, ax in zip(dict.fromkeys(r['turn'] for r in raw), axes.flat):
        samples = [r for r in raw if r['turn']==label]
        predictions = {r['ms_from_last_edge']:r for r in replay if r['turn']==label}
        def field(key):
            return np.array([float(r[key]) for r in samples])
        t = field('ms_from_last_edge')/1000
        speed = (abs(field('left_mmps'))+abs(field('right_mmps')))/2
        steady = speed[(t>=-.50)&(t<=-.25)].mean()
        factor = (1 if label.startswith('ccw') else -1)/np.cos(field('head_rad'))
        traces = {'measured':np.degrees(field('head_gz'))}
        for method in methods:
            traces[method] = np.array([float(predictions[r['ms_from_last_edge']][method]) for r in samples])
        for name in traces:
            rate = traces[name] * factor
            traces[name] = rate - np.median(rate[t>.25])
        for fraction in (.6,.8,.9):
            k = np.flatnonzero((speed[:-1]>=fraction*steady)&(speed[1:]<fraction*steady))[-1]
            start = np.interp(fraction*steady,speed[k:k+2][::-1],t[k:k+2][::-1])
            pts = np.r_[start,t[(t>start)&(t<start+.08)],start+.08]
            angles = {}
            for name, rate in traces.items():
                y = np.interp(pts,t,rate)
                cumulative = np.r_[0,np.cumsum((y[1:]+y[:-1])*.5*np.diff(pts))]
                angles[name] = cumulative[-1]
                if fraction==.8:
                    ax.plot(1000*(pts-start),cumulative,label=name)
            for name in methods:
                err = angles[name]-angles['measured']
                writer.writerow([label,fraction,name,angles['measured'],angles[name],err,100*err/angles['measured']])
        ax.set(title=label,xlabel='ms from 80% wheel speed',ylabel='Accumulated yaw (degrees)')
axes[0,0].legend(fontsize=7)
fig.savefig(here/'native-brake-angle.png',dpi=160)

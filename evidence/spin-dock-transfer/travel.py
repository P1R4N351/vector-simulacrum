"""Compare physical count travel, simulated shaft positions, and head-gyro angle."""
from pathlib import Path
import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import cumulative_trapezoid

here = Path(__file__).resolve().parent
def read(path):
    return np.genfromtxt(path, names=True, delimiter=',', dtype=None, encoding='utf8')

hardware = read(here.parent/'hw-rec-20261119/unit2-native-replay-input.csv')
sim = read(here/'travel.csv')
count_mm = .96 * 29 * .25 * np.pi / 172.3
scores = []
fig, axes = plt.subplots(2, 2, figsize=(10,6), layout='constrained')
for spin in ('ccw_1','cw_1','ccw_2','cw_2'):
    h = hardware[hardware['turn']==spin]
    sign = 1 if spin.startswith('ccw') else -1
    t = h['ms_from_last_edge']/1000
    assert np.all(np.diff(t)>0) and np.max(np.diff(t)) < .012 and t[-1]>.4
    grid = np.r_[t[t<.4], .4]
    series = [('Physical', t, h['left_count']*count_mm, -h['right_count']*count_mm, np.degrees(h['head_gz']))]
    for label in ('canonical','shaft_reported','shaft_counts','shaft_count_position'):
        r = sim[(sim['turn']==spin)&(sim['variant']==label)]
        series.append((label,r['ms']/1000,r['shaft_left_mm'],r['shaft_right_mm'],r['gyro_dps']))
    for label, time, left, right, gyro in series:
        wheel = sign*np.interp(grid,time,((right-right[0])-(left-left[0]))/2)
        angle = sign*cumulative_trapezoid(np.interp(grid,time,gyro),grid,initial=0)
        scores.append((spin,label,wheel[-1],angle[-1],angle[-1]/wheel[-1]))
        if spin.endswith('_2'):
            a,b = axes[0 if sign==1 else 1]
            a.plot(grid,wheel,label=label.replace('_',' '))
            b.plot(wheel,angle,label=label.replace('_',' '))
            a.set_ylabel(f'{spin.upper()} wheel travel (mm)')
            a.set_xlabel('Seconds from last encoder edge')
            b.set_ylabel('Integrated head gyro (degrees)')
            b.set_xlabel('Differential wheel travel / 2 (mm)')
axes[0,0].legend(fontsize=8)
fig.savefig(here/'travel-angle.png',dpi=165)
with (here/'travel-scores.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['spin','source','differential_half_mm','head_deg','head_deg_per_mm']);w.writerows(scores)

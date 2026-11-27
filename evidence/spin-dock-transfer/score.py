from pathlib import Path
import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import cumulative_trapezoid

here = Path(__file__).resolve().parent
def read(path):
    return np.genfromtxt(path, delimiter=',', names=True, dtype=None, encoding='utf8')

hardware = read(here.parent/'hw-rec-20261119/unit2-native-replay-input.csv')
sim = read(here/'replay.csv')
fig, axes = plt.subplots(2, 2, figsize=(10, 6), sharex=True, layout='constrained')
scores = []
for ax, name in zip(axes.flat, ['ccw_1','cw_1','ccw_2','cw_2']):
    h = hardware[hardware['turn']==name]
    sign = 1 if name.startswith('ccw') else -1
    t = h['ms_from_last_edge']/1000
    assert np.all(np.diff(t)>0) and np.max(np.diff(t))<.012 and t[-1]>.4
    series = [('Physical', t, np.degrees(h['head_gz']), h['left_mmps'], h['right_mmps'])]
    for key in ['canonical','driven_rear_envelope','canonical_no_queue','driven_rear_envelope_no_queue']:
        r = sim[(sim['turn']==name)&(sim['variant']==key)]
        series.append((key, r['ms']/1000, r['gyro_dps'], r['left_mmps'], r['right_mmps']))
    for label, time, gyro, left, right in series:
        # Use exactly the same physical interval, interpolating the endpoint.
        grid = np.r_[t[t<.4], .4]
        rate = sign*np.interp(grid,time,gyro)
        angle = cumulative_trapezoid(rate,grid,initial=0)
        steady = (grid>=-.85)&(grid<=-.25)
        speed = (abs(np.interp(grid,time,left))+abs(np.interp(grid,time,right)))/2
        scores.append([name,label,angle[-1],np.mean(rate[steady]),np.mean(speed[steady]),angle[-1]-np.interp(0,grid,angle)])
        ax.plot(grid,angle,ls='--' if label.endswith('_no_queue') else '-',label=label.replace('_',' '))
    ax.axvline(0,color='0.5',ls=':',lw=1)
    ax.set_title(name.replace('_',' ').upper()); ax.set_ylabel('Integrated head gyro (°)')
for ax in axes[-1]: ax.set_xlabel('Seconds from last physical encoder edge')
axes[0,0].legend(fontsize=8)
fig.savefig(here/'spin-angle.png',dpi=160)
with (here/'scores.csv').open('w') as output:
    writer=csv.writer(output);writer.writerow(['spin','source','total_deg','steady_dps','steady_wheel_mmps','post_edge_deg']);writer.writerows(scores)

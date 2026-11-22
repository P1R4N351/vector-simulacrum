"""Compare the first unloaded undock raise with the held-out R11 roll raise."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent


def crossing(times, heights, level):
    i = np.flatnonzero(heights >= level)[0]
    assert i > 0 and times[i] - times[i-1] < .15
    fraction = (level-heights[i-1]) / (heights[i]-heights[i-1])
    return times[i-1] + fraction*(times[i]-times[i-1])


def main():
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), layout='constrained', sharex=True)
    with (HERE/'lift-vibration.csv').open('w') as f:
        w = csv.writer(f, lineterminator='\n')
        w.writerow(['run','rise_mm','duration_10_90_s','speed_mm_s','max_wheel_mm_s','slots','rest_sd_mg','rise_sd_mg','tap_range'])
        for run in ['W1','W4','W5','W8','R5','R6','R11']:
            folder = HERE.parent/run
            action = 'roll cube' if run == 'R11' else 'undock'
            with (folder/'actions.csv').open() as af:
                start = float(next(r['sec'] for r in csv.DictReader(af) if r['kind']=='begin' and r['action']==action))
            s = np.genfromtxt(folder/'state.csv.gz', names=True, delimiter=',')
            c = np.genfromtxt(folder/'cube.csv.gz', names=True, delimiter=',')
            t = s['sec']-start
            early = t[(t>=0)&(t<(1.5 if run=='R11' else 3.35))]
            h = np.interp(early,t,s['lift'])
            base = np.median(h[early < .2])
            travel = np.median(h[early > early[-1]-.2])-base
            first,last = [crossing(early,h,base+travel*p) for p in (.1,.9)]
            robot = (t>=first-.05)&(t<=last+.05)
            wheel = max(abs(s[k][robot]).max() for k in ['left','right'])
            tc = c['sec']-start-first
            rest = (tc>-.8)&(tc<-.15)
            active = (tc>=0)&(tc<=last-first)
            mag = np.linalg.norm(np.column_stack([c[k] for k in 'xyz']),axis=1)/8192*1000
            assert wheel <= 1 and rest.sum() >= 12 and active.sum() >= 12
            w.writerow([run,travel,last-first,.8*travel/(last-first),wheel,active.sum(),mag[rest].std(),mag[active].std(),np.ptp(c['tap'][active])])
            color = '#b83726' if run=='R11' else '#34788a'
            label = run if run=='R11' else ('Undock raises' if run=='W1' else None)
            axes[0].plot(t-first,s['lift'],color=color,alpha=.75,label=label)
            centers = np.arange(-.65,1,.1)
            spread = [mag[(tc>=a-.05)&(tc<a+.05)].std() if ((tc>=a-.05)&(tc<a+.05)).sum()>=6 else np.nan for a in centers]
            axes[1].plot(centers,spread,color=color,alpha=.75)
    axes[0].set(ylabel='Lift height (mm)', xlim=(-.7,1))
    axes[0].legend()
    axes[1].set(ylabel='Cube acceleration SD (mg)',xlabel='Seconds from 10% of upward travel')
    fig.savefig(HERE/'lift-vibration.png',dpi=150)


if __name__ == '__main__':
    main()

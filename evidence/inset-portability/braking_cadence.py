"""Decimate native stops on the controller frame clock, without fitting a replay."""
import csv
from itertools import groupby
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
source = here.parent / 'hw-rec-20261119' / 'unit2-native-stops.csv'


def measure(time, speed, rate, threshold):
    # Last descending crossing before the final edge or first SDK frame after it.
    crossings = np.where((speed[:-1] >= threshold) & (speed[1:] < threshold))[0]
    i = crossings[-1]
    start = time[i] + (time[i+1]-time[i])*(threshold-speed[i])/(speed[i+1]-speed[i])
    return start, integrate(time, rate, start)


def integrate(time, rate, start):
    end = start + .3
    assert time[0] <= start < end <= time[-1]
    points = np.r_[start, time[(time > start) & (time < end)], end]
    return np.degrees(np.trapezoid(np.interp(points, time, rate), points))


fig, panels = plt.subplots(1, 2, figsize=(9, 3.8), layout='constrained')
with source.open() as stream, (here/'braking-cadence.csv').open('w', newline='') as out:
    writer = csv.writer(out)
    writer.writerow(['spin','frame_phase','native_angle_deg','sampled_angle_deg','angle_error_deg',
                     'crossing_error_ms','largest_frame_gap_ms','anchor_only_error_deg','gyro_sampling_error_deg'])
    for label, group in groupby(csv.DictReader(stream), key=lambda r: r['turn']):
        rows = list(group)
        def field(k):
            return np.array([float(r[k]) for r in rows])
        t = field('ms_from_last_edge') / 1000
        f = field('frame').astype(int)
        speed = (abs(field('left_mmps')) + abs(field('right_mmps')))/2
        g = field('head_gz')/np.cos(field('head_rad'))
        g = (g-np.median(g[t>.25]))*(1 if label.startswith('ccw') else -1)
        threshold = .8*np.mean(speed[(t>=-.50)&(t<=-.25)])
        base_t, base_a = measure(t,speed,g,threshold)
        dt, da = [], []
        for phase in range(12):
            take = f % 12 == phase
            start, angle = measure(t[take],speed[take],g[take],threshold)
            dt.append(1000*(start-base_t))
            da.append(angle-base_a)
            dense_shifted = integrate(t, g, start)
            writer.writerow([label,phase,base_a,angle,da[-1],dt[-1],1000*np.max(np.diff(t[take])),
                             dense_shifted-base_a, angle-dense_shifted])
        panels[0].plot(range(12),dt,'.-',label=label)
        panels[1].plot(range(12),da,'.-',label=label)
        print(label, 'native angle',round(base_a,2),'angle error range',np.round([min(da),max(da)],2),
              'crossing error ms',np.round([min(dt),max(dt)],1))
for ax, label in zip(panels,('Wheel crossing error (ms)','Integrated yaw error (degrees)')):
    ax.set(xlabel='Retained frame phase (modulo 12)',ylabel=label)
    ax.axhline(0,color='gray',lw=.8)
panels[1].legend()
fig.savefig(here/'braking-cadence.png',dpi=160)

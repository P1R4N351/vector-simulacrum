"""Overlay the camera with the native references checked against browser WASM."""
import json

import matplotlib.pyplot as plt
import numpy as np
from release import crossing, here

t, a = np.loadtxt(here/'native-face-angle.csv', delimiter=',', skiprows=1).T
high = np.nanmedian(a[(t > 5.15) & (t < 5.23)])
low = np.nanmedian(a[(t > 6) & (t < 10)])
keep = (t >= 5.19) & (t <= 5.45)
t, tilt = t[keep], (a[keep]-low)/(high-low)
fig, ax = plt.subplots(figsize=(7, 4), layout='constrained')
ax.plot(1000*(t-crossing(t, tilt, .9)), tilt, 'o-', label='S22B camera, projected glyph')
runs = json.loads((here.parents[2]/'web/test/fixtures/cube-landing.json').read_text())
for run in runs:
    t, up = np.array(run['trace']).T[:2]
    tilt = np.rad2deg(np.arccos(up.clip(-1, 1)))/15.76
    label = 'Simulation, eight overlapping releases' if run is runs[0] else None
    ax.plot(1000*(t-crossing(t, tilt, .9)), tilt, color='#a84925', alpha=.55, label=label)
ax.set(xlim=(-30, 120), xlabel='Milliseconds from 90% remaining tilt',
       ylabel='Fraction of tilt remaining', title='Cube settling on the shelf')
ax.legend()
fig.savefig(here/'browser-landing.png', dpi=150)

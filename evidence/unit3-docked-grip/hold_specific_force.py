"""Resolve within-packet specific-force changes parallel and transverse to gravity."""
import csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
groups = np.loadtxt(root/'cube.csv',delimiter=',',skiprows=1).reshape(-1,3,5)
assert np.all(groups[:,:,1] == [0,1,2])
times = groups[:,0,0]
force = groups[:,:,2:] / 8192
mag = np.linalg.norm(force,axis=2)
# Rotation alone, with no translational acceleration, preserves gravity magnitude.
# Bias, scale anisotropy and vibration can all affect a physical chip's readings.
steps = np.linalg.norm(np.diff(force,axis=1),axis=2)
radial = np.abs(np.diff(mag,axis=1))
assert np.all(radial <= steps+1e-9)
with (root/'release-rotation.csv').open() as f:
    optical = list(csv.DictReader(f))
windows = [('Before command',-.21,0),('Raised hold',1.15001,1.37309),('After settling',2.5,3.25)]
rows = []
fig,axes = plt.subplots(2,1,figsize=(8,5),sharex=True,layout='constrained')
axes[0].plot(times,1000*radial.max(axis=1),'.',label='Magnitude change')
axes[0].plot(times,1000*steps.max(axis=1),'+',alpha=.65,label='Vector change')
axes[0].set(yscale='symlog',ylabel='Largest adjacent-slot change (mg)')
for view,mult in [('A',1),('B',-1)]:
    selected=[r for r in optical if r['view']==view]
    axes[1].plot([float(r['seconds']) for r in selected],
                 [mult*float(r['image_deg']) for r in selected],'.-',label='View '+view)
axes[1].set(xlabel='Seconds from lift command',ylabel='Projected glyph rotation (°)\nopposed views sign-aligned',xlim=(-.22,3.25))
for ax in axes:
    ax.axvspan(windows[1][1],windows[1][2],color='orange',alpha=.16)
    ax.legend(fontsize=8)
fig.savefig(root/'hold-specific-force.png',dpi=165)
for label,lo,hi in windows:
    mask=(times>=lo)&(times<=hi)
    a,b=radial[mask]*1000,steps[mask]*1000
    rows.append([label,int(mask.sum()),np.median(a),a.max(),np.median(b),b.max(),mag[mask].min(),mag[mask].max()])
quiet=(times<0)|(times>=2.5)
hold=(times>=windows[1][1])&(times<=windows[1][2])
print('Hold packets beyond quiet max',sum(radial[hold].max(axis=1)>radial[quiet].max()),'/',hold.sum())
with (root/'hold-specific-force.csv').open('w') as f:
    w=csv.writer(f)
    w.writerow('window packets median_magnitude_step_mg max_magnitude_step_mg median_vector_step_mg max_vector_step_mg min_g max_g'.split())
    w.writerows(rows)

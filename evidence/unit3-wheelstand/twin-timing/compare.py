"""Pair mutual nearest camera timestamps without reusing frames."""
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
fig, axes = plt.subplots(2, 1, figsize=(8,5), layout='constrained')
for ax, name in zip(axes, ['first','repeat']):
    frames = []
    for side in ['a','b']:
        path = here / f'{name}-{side}.pts'
        assert '#tb 0: 1/1000000' in path.read_text().splitlines()
        ts = np.loadtxt(path,delimiter=',',usecols=2,dtype=int)
        assert len(ts)==100 and np.all(np.diff(ts)>0)
        frames.append(ts)
    a,b = frames
    distances = abs(a[:,None]-b)
    near = distances.argmin(axis=1)
    keep = distances.argmin(axis=0)[near]==np.arange(len(a))
    delta = (b[near[keep]]-a[keep])/1000
    ax.plot((a[keep]-a[0])/1e6,delta,'.')
    ax.axhline(0,c='gray',lw=.6)
    ax.set(title=name.capitalize()+' burst',ylabel='B minus A (ms)')
    print(name,'mutual pairs',sum(keep),'fps',*[round(99e6/(t[-1]-t[0]),2) for t in frames],
          'absolute skew median/p95/max (ms)',np.round(np.percentile(abs(delta),[50,95,100]),2))
axes[-1].set_xlabel('Seconds since first A frame')
fig.savefig(here/'timing.png',dpi=150)

"""Compare 19-ray geometric interception with physical minimum-range observations."""
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
beam = np.genfromtxt(here / 'fork-beam.csv', delimiter=',', names=True, dtype=None, encoding='utf-8')
samples = np.genfromtxt(here / 'samples.csv', delimiter=',', names=True)
fig, axes = plt.subplots(2, 1, figsize=(8, 6), layout='constrained')
for name in ('boxes', 'printed'):
    g = beam[beam['fork'] == name]
    axes[0].plot(g['lift_mm'], g['ray_hits'], label=name)
axes[0].axvspan(34, 56, alpha=.09, color='black', label='firmware rejection window')
axes[0].set(xlim=(30, 60), ylabel='Rays striking fork (of 19)', xlabel='Reported lift height (mm)')
axes[0].legend()
g = beam[beam['fork'] == 'printed']
for delay, marker in [(0, 'o'), (.071, 'x')]:
    hits, physical = [], []
    for trial in (3, 4, 5):
        path = samples[samples['trial'] == trial]
        rows = path[(path['lift_mm'] >= 34) & (path['lift_mm'] <= 56)]
        heights = np.interp(rows['seconds'] - delay, path['seconds'], path['lift_mm'])
        idx = np.abs(g['lift_mm'][:, None] - heights).argmin(axis=0)
        hits.extend(g['ray_hits'][idx])
        physical.extend(rows['real_mm'])
    axes[1].scatter(hits, physical, marker=marker, label=f'{int(delay * 1000)} ms delay', alpha=.8)
    print(int(delay * 1000), list(zip(hits, physical)))
axes[1].set(xlabel='Printed fork rays intercepted at sample time', ylabel='P2Y8 range (mm)',
            title='Nine in-window readings from push trials 3–5', xticks=[0, 4, 8, 12, 16, 19])
axes[1].legend()
fig.savefig(here / 'fork-beam.png', dpi=160)

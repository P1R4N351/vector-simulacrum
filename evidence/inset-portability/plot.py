"""Plot steady turn and pivot errors for frozen insets across solver versions."""
from pathlib import Path
import csv
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
fig, axes = plt.subplots(1, 2, figsize=(9, 4), layout='constrained')
for filename in ('arm64-current-3.11.csv', 'x86_64-3.11.csv', 'arm64.csv'):
    rows = list(csv.DictReader((here/filename).open()))
    x = sorted(set(float(r['inset_mm']) for r in rows))
    rms, pivot = [], []
    for inset in x:
        g = [r for r in rows if float(r['inset_mm']) == inset]
        rms.append(np.sqrt(np.mean([(100*(float(r['yaw_ratio'])-1))**2 for r in g])))
        pivot.append(np.mean([float(r['pivot_x_mm']) for r in g]))
    for ax, y in zip(axes, (rms, pivot)):
        ax.plot(x, y, 'o', label=rows[0]['platform']+' · '+rows[0]['mujoco'])
        ax.set(xlabel='Inset per end (mm)', xticks=x, xticklabels=['3.10','3.242','3.75'])
    print(rows[0]['mujoco'], rms, pivot)
axes[0].set_ylabel('P2Y8 steady yaw RMS relative error (%)')
axes[1].set_ylabel('Mean pivot x from front axle (mm)')
axes[1].axhline(-27.30, ls='--', color='gray', label='Camera mean')
axes[0].legend()
axes[1].legend()
fig.savefig(here/'solver-transfer.png', dpi=160)

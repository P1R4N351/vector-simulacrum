"""Plot the physical and relayed Z words near each run's largest Z discrepancy."""
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

out = Path(__file__).resolve().parent
fig, axes = plt.subplots(2, 1, figsize=(9, 6), layout='constrained')
for ax, run in zip(axes, ('R5', 'R8')):
    before, after = (np.genfromtxt(f'{rev}-{run}.csv', delimiter=',', names=True)
                     for rev in ('before','after'))
    assert np.array_equal(before['physical_z'], after['physical_z'])
    peak = np.argmax(np.abs(before['physical_z']-before['relayed_z']))
    window = slice(max(0,peak-17),peak+20)
    for data,key,label,style in ((before,'relayed_z','Previous relay','.--'),
                                 (after,'relayed_z','Corrected relay','-'),
                                 (after,'physical_z','Physical BLE words','x')):
        ax.plot(data['sample'][window], data[key][window],style,label=label)
    ax.set(title=run, xlabel='Accelerometer sample index', ylabel='Z (signed counts)')
axes[0].legend()
fig.savefig(out/'wire-report-overlay.png',dpi=160)

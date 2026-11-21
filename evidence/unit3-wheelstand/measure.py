"""Plot shelf wheelstand telemetry using pod receipt times and original BLE packet order."""
import gzip
from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
for trial in sorted(root.glob('W*')):
    state = np.genfromtxt(trial/'state.csv.gz', delimiter=',', names=True)
    cube = np.genfromtxt(trial/'cube.csv.gz', delimiter=',', names=True)
    actions = list(csv.DictReader((trial/'actions.csv').open()))
    fig, axes = plt.subplots(3,1,figsize=(9,6),sharex=True,layout='constrained')
    t = state['sec']
    axes[0].plot(t,np.rad2deg(state['pitch']),label='Chassis pitch')
    axes[0].plot(t,np.rad2deg(state['head']),label='Head angle',alpha=.7)
    axes[0].set_ylabel('Angle (°)')
    axes[1].plot(t,state['left'],label='Left wheel')
    axes[1].plot(t,state['right'],label='Right wheel',alpha=.7)
    axes[1].set_ylabel('Wheel speed (mm/s)')
    forces=np.column_stack([cube[k] for k in ('x','y','z')])/8192
    axes[2].plot(cube['sec'],np.linalg.norm(forces,axis=1),lw=.65,label='Cube accelerometer')
    axes[2].set(ylabel='Specific force (g)',xlabel='Seconds from undock command (pod receipt)')
    for ax in axes:
        ax.legend(loc='upper right')
        for row in actions:
            if row['action']=='pop_a_wheelie':
                ax.axvline(float(row['sec']),color='k',ls=':',lw=1)
    fig.savefig(trial/'traces.png',dpi=145)
    plt.close(fig)

from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt

root = Path(__file__).parent
data = np.genfromtxt(root / 'replayed.csv', delimiter=',', names=True)
fig, axes = plt.subplots(4, 2, figsize=(9, 9), sharex=True, layout='constrained')
metrics = []
for trial in range(1, 9):
    rows = data[data['trial'] == trial]
    source = int(rows['from_deg'][0])
    direction = np.sign(8 - source)
    start = rows['sim_initial_deg'][0]
    opposite = max(0, -np.min(direction * (rows['real_deg'] - start)))
    for col, signal in enumerate(('deg', 'gyro_y')):
        ax = axes[[-4, 20, -16, 32].index(source), col]
        for kind, color in [('real', 'teal'), ('sim', 'darkorange')]:
            ax.plot(rows['s'], rows[kind+'_'+signal], '.-' if kind=='real' else '--',
                    color=color, alpha=.85, markersize=3)
        ax.set_title(f'{source}° to 8° · '+('encoder (°)' if col==0 else 'gyro Y (rad/s)'))
    metrics.append([trial, source, rows['real_deg'][-1], rows['sim_deg'][-1], opposite])
for ax in axes[-1]:
    ax.set_xlabel('Seconds after command')
axes[0,0].legend(['Physical', 'Canonical sim'])
fig.savefig(root/'replay-overlay.png', dpi=160)
with (root/'endpoint-errors.csv').open('w') as f:
    w = csv.writer(f, lineterminator='\n')
    w.writerow(['trial','start_target_deg','real_last_deg','sim_last_deg','opposite_excursion_from_sim_start_deg'])
    w.writerows(metrics)
for group in ([12],[24],[12,24]):
    selected = np.array([m for m in metrics if abs(8-m[1]) in group])
    print(group, np.mean(abs(selected[:,2]-selected[:,3])))

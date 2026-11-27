"""Score the first repeat per start and independently report the second repeats."""
from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
torques = [0, .05, .1, .2, .3, .4, .5, 1, 2, 3, 4, 6, 8, 12]
def read(name):
    return np.genfromtxt(root / name, delimiter=',', names=True)

baseline = read('replayed.csv')
runs = {torque: read(f'released-{torque:g}mNm.csv') for torque in torques}
def summarize(run, ids):
    endpoints, speeds, released = [], [], []
    for i in ids:
        rows = run[run['trial'] == i]
        endpoints.append(abs(rows['sim_deg'][-1] - rows['real_deg'][-1]))
        speeds.append(np.mean((rows['sim_gyro_y'] - rows['real_gyro_y'])**2))
        released.append(rows['released'][-1] if 'released' in rows.dtype.names else 0)
    return [np.mean(endpoints), np.sqrt(np.mean(speeds)), int(sum(released))]

groups = {'first': [1,2,3,4], 'second': [5,6,7,8]}
scores = []
for label, ids in groups.items():
    for condition, run in [('canonical', baseline), *runs.items()]:
        scores.append([label,condition,*summarize(run,ids)])
with (root/'friction-scores.csv').open('w') as f:
    writer = csv.writer(f, lineterminator='\n')
    writer.writerow(['repeat','friction_mNm_or_baseline','endpoint_mae_deg','gyro_rmse_rad_s','released_by_last_sample'])
    writer.writerows(scores)
eligible = [r for r in scores if r[0]=='first' and r[1]!='canonical' and r[4]==4]
selected = min(eligible,key=lambda r:r[2])[1]
print('Selected torque:', selected)
fig, axes = plt.subplots(2,2,figsize=(10,6),layout='constrained')
for ax,i in zip(axes.flat,groups['second']):
    for run,c,key,label in [(baseline,'black','real_deg','Physical'),(baseline,'darkorange','sim_deg','Canonical'),
                            (runs[selected],'teal','sim_deg',f'Release + {selected:g} mN·m friction')]:
        rows=run[run['trial']==i]
        ax.plot(rows['s'],rows[key],'.-',c=c,label=label)
    ax.set_title(f"Held-out trial {i} · {rows['from_deg'][0]:g}° to 8°")
    ax.set(xlabel='Time from command (s)',ylabel='Published head angle (°)')
axes[0,0].legend(fontsize=8)
fig.savefig(root/'friction-holdout.png',dpi=155)
with (root/'posture-friction-scores.csv').open('w') as f:
    writer = csv.writer(f, lineterminator='\n')
    writer.writerow(['condition','endpoint_mae_deg','gyro_rmse_rad_s','released_by_last_sample'])
    for name in ['replayed','released-0.5mNm','released-6mNm']:
        writer.writerow([name,*summarize(read(f'posture-{name}.csv'), range(1,9))])

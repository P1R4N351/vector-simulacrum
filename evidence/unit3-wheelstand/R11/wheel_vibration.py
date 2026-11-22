"""Fit shelf-cube vibration on R10 search turns and transfer to R11."""
from pathlib import Path
import csv

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent


def samples(run):
    folder = HERE.parent / run
    with (folder / 'actions.csv').open() as f:
        actions = list(csv.DictReader(f))
    cube = np.genfromtxt(folder / 'cube.csv.gz', names=True, delimiter=',')
    state = np.genfromtxt(folder / 'state.csv.gz', names=True, delimiter=',')
    time = state['sec']
    assert np.all(np.diff(time) >= 0)
    speed = np.maximum(abs(state['left']), abs(state['right']))
    magnitude = np.linalg.norm(np.column_stack([cube[k] for k in 'xyz']), axis=1) / 8192
    rows = []
    for start, end in zip(actions[::2], actions[1::2]):
        name = start['action']
        assert start['kind'] == 'begin' and end['kind'] == 'end' and end['action'] == name
        if name not in ('aim', 'roll cube', 'passive post-stop capture'):
            continue
        for a in np.arange(float(start['sec']), float(end['sec']) - .249999, .25):
            take = (cube['sec'] >= a) & (cube['sec'] < a + .25)
            if take.sum() < 12:
                continue
            times = cube['sec'][take]
            rows.append((name, a, np.mean(np.interp(times, time, speed)**2),
                         magnitude[take].copy()))
    return rows


def main():
    runs = {key: samples(key) for key in ('R10', 'R11')}
    idle = np.concatenate([m for name, _, _, m in runs['R10'] if name.startswith('passive')])
    center = idle.mean()
    floor = idle.var() * 1e6
    def observed(m):
        return np.mean(((m-center)*1000)**2)
    turns = [(speed, observed(m)) for name, _, speed, m in runs['R10'] if name == 'aim']
    x, y = np.transpose(turns)
    slope = max(0., np.dot(x, y-floor) / np.dot(x, x))
    fig, axes = plt.subplots(1, 2, figsize=(9, 4), sharey=True, layout='constrained')
    with (HERE/'wheel-vibration.csv').open('w') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(['run','action','start_s','wheel_rms_mm_s','measured_mg','rest_prediction_mg','motion_prediction_mg'])
        for ax, (key, rows) in zip(axes, runs.items()):
            for name in ('aim','roll cube','passive post-stop capture'):
                group = [(a,x,observed(m)) for n,a,x,m in rows if n == name]
                if not group:
                    continue
                a,x,y = np.transpose(group)
                pred = np.sqrt(floor+slope*x)
                ax.scatter(np.sqrt(x),np.sqrt(y),s=13,alpha=.7,label=name)
                writer.writerows(zip([key]*len(x),[name]*len(x),a,np.sqrt(x),np.sqrt(y),np.repeat(np.sqrt(floor),len(x)),pred))
                print(key,name,len(x), 'RMSE rest and motion',np.sqrt(np.mean((np.sqrt(y)-np.sqrt(floor))**2)),np.sqrt(np.mean((np.sqrt(y)-pred)**2)))
            v = np.linspace(0,ax.get_xlim()[1])
            ax.plot(v,np.sqrt(floor+slope*v*v),color='black',label='R10 fit')
            ax.set(title=key,xlabel='Wheel speed RMS (mm/s)')
            ax.legend(fontsize=8)
    axes[0].set_ylabel('Cube acceleration RMS about rest (mg)')
    fig.savefig(HERE/'wheel-vibration.png',dpi=150)
    print('floor and slope', floor, slope)


if __name__ == '__main__':
    main()

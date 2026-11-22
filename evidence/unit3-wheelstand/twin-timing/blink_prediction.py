"""Predict autonomous blink edges from the separate stationary packet recording."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'unit3-cube-accelerometer'))
from packet_cadence import clocks, estimate


def optical_edges(trace):
    time, level = trace['relative_s'], trace['green_contrast']
    early = (time > .05) & (time < .8)
    threshold = np.percentile(level[early], [10, 90]).mean()
    bright = level > threshold
    indices = np.flatnonzero(np.diff(bright) & (time[:-1] > .05) & (time[1:] < 8))
    assert not bright[indices[0]+1], 'First observed transition must be falling'
    return time[indices], time[indices+1], bright[indices+1]


def main():
    rest = clocks('resting')[0]
    normal, gap, _ = estimate(rest)
    returns = rest[1:][np.diff(rest) > .15]
    period = np.polyfit(np.arange(len(returns)), returns, 1)[0]
    pause = gap - normal
    first_return = np.genfromtxt(HERE/'blink-clock-gaps.csv', names=True, delimiter=',')['end_s'][0]
    traces = np.genfromtxt(HERE/'blink-clock-traces.csv', names=True, delimiter=',', dtype=None, encoding=None)
    edges = {cam: optical_edges(traces[traces['camera']==cam]) for cam in 'AB'}
    anchor = (edges['A'][0][0] + edges['A'][1][0])/2
    print(f'rest period={period:.6f} excess pause={pause:.6f} phase={anchor:.4f}')
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), layout='constrained', sharex=True)
    with (HERE/'blink-prediction.csv').open('w') as out:
        writer = csv.writer(out, lineterminator='\n')
        writer.writerow(['camera','model','ordinal','rising','observed_left_s','observed_right_s','predicted_s'])
        for (cam, (left, right, rising)), ax in zip(edges.items(), axes):
            index = np.arange(len(left))
            nominal = anchor + .150*index
            delayed = nominal.copy()
            for end in np.arange(first_return, 9, period):
                delayed[delayed >= end-pause] += pause
            predictions = {'Continuous': nominal, 'Uniform slowdown': anchor + .150*index*period/(period-pause),
                           'Periodic pause': delayed}
            mid = (left+right)/2
            ax.fill_between(mid, (left-mid)*1000, (right-mid)*1000, color='0.8', label='Camera frame bracket')
            for name, predicted in predictions.items():
                error = predicted-mid
                score = left > first_return
                print(cam, name, score.sum(), 'median absolute error ms', round(np.median(abs(error[score]))*1000,1),
                      'inside bracket', np.sum(score & (predicted>=left) & (predicted<=right)))
                ax.plot(mid, error*1000, '.-', label=name, lw=1)
                writer.writerows(zip([cam]*len(index), [name]*len(index), index, rising.astype(int),left,right,predicted))
            ax.axvline(first_return, color='0.4', ls=':')
            ax.set(ylabel='Timing error (ms)', title=f'Camera {cam}')
    axes[0].legend(ncol=2)
    axes[1].set_xlabel('Seconds from animation start acknowledgement')
    fig.savefig(HERE/'blink-prediction.png', dpi=160)


if __name__ == '__main__':
    main()

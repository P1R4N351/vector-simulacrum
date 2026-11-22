"""Predict accelerometer receipt counts in short windows on independent roll captures."""
from pathlib import Path
import csv
import sys

import numpy as np
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'unit3-cube-accelerometer'))
from packet_cadence import clocks, estimate


def evaluate(offset=0.0):
    train = clocks('resting')[0]
    normal, _, _ = estimate(train)
    indices = np.flatnonzero(np.diff(train) > .15) + 1
    period = np.polyfit(np.arange(len(indices)), train[indices], 1)[0]
    mean_count = np.mean(np.diff(indices))
    rows = []
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), layout='constrained')
    for run in ('R5','R6','R7','R8','R9'):
        data = np.genfromtxt(HERE.parent/run/'cube.csv.gz', names=True, delimiter=',')
        times = data['uptime'][data['slot']==0]
        first = np.flatnonzero(np.diff(times) > .15)[0]+1
        times = times[first:] - times[first]
        # Ignore the phase-setting cycle and discard any partial final window.
        bins = np.arange(period + offset, times[-1], .15)
        def counts(t):
            return np.diff(np.searchsorted(t, bins, side='left'))
        observed = counts(times)
        n = np.arange(int(times[-1]/normal)+2)
        # Map fractional packet ordinal into a cycle. The first observed return
        # has ordinal zero. Use rest's mean count without rounding each cycle.
        phase = n/mean_count
        periodic = np.floor(phase)*period + (phase%1)*mean_count*normal
        models = {'Nominal': n*normal, 'Mean rate': n*period/mean_count, 'Periodic': periodic}
        for name,t in models.items():
            predicted = counts(t)
            err = predicted-observed
            rows.append((run,name,len(err),np.mean(abs(err)),np.mean(err==0),
                         np.sum(observed==0),np.sum((predicted==0)&(observed==0)),np.sum(predicted==0)))
            if run=='R8' and name!='Nominal':
                axes[0].step(bins[:-1], predicted, where='post',label=name,alpha=.75)
        if run=='R8':
            axes[0].plot(bins[:-1]+.075,observed,'k.',label='Recorded')
    for name in models:
        group=[r for r in rows if r[1]==name]
        axes[1].plot([r[0] for r in group],[r[3] for r in group],'.-',label=name)
    axes[0].set(xlabel='Seconds after first gap return in R8',ylabel='Packets per 150 ms')
    axes[1].set(ylabel='Mean absolute count error',xlabel='Held-out capture')
    for ax in axes:
        ax.legend()
    suffix = '-shifted' if offset else ''
    fig.savefig(HERE/f'packet-windows{suffix}.png',dpi=150)
    plt.close(fig)
    with (HERE/f'packet-windows{suffix}.csv').open('w') as out:
        w=csv.writer(out,lineterminator='\n')
        w.writerow(['run','model','windows','mae_packets','exact_fraction','observed_empty','correct_empty','predicted_empty'])
        w.writerows(rows)
    for name in models:
        a=np.asarray([r[2:] for r in rows if r[1]==name])
        print(name,a[:,0].sum(),np.average(a[:,1],weights=a[:,0]),np.average(a[:,2],weights=a[:,0]),a[:,3:].sum(axis=0))


if __name__=='__main__':
    evaluate()
    evaluate(.075)

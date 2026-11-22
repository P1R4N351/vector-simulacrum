"""Anchor the awake supervisor once per continuously observed run of cube callbacks."""
import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent
period = 30.48  # Frozen from separate run S (254 ms over 50 loops; awake uses six).
kinds = {'object_tapped','object_moved','object_stopped_moving','object_up_axis_changed'}


def main():
    results = []
    for path in sorted((root.parent/'unit3-wheelstand').rglob('events.csv')):
        states = path.with_name('state.csv.gz')
        if not states.exists():
            continue
        with gzip.open(states,'rt') as f:
            records = list(csv.DictReader(f))
        if not records:
            continue
        times = np.array([float(r['sec']) for r in records])
        awake = np.array([not (int(r['status']) & 0x400) for r in records])
        assert np.all(np.diff(times)>=0)
        anchor = None
        seen = set()
        with path.open() as f:
            for r in csv.DictReader(f):
                if r['event'] not in kinds:
                    continue
                t, stamp = float(r['sec']), int(r['timestamp'])
                i = np.searchsorted(times,t)
                if not 0<i<len(times) or times[i]-times[i-1]>.3 or not awake[i-1:i+1].all():
                    anchor = None
                    continue
                if stamp in seen:
                    continue
                seen.add(stamp)
                if anchor is not None:
                    j = anchor[2]
                    if i<j or not awake[j-1:i+1].all() or np.diff(times[j-1:i+1]).max()>.3:
                        anchor = None
                if anchor is None:
                    anchor = t,stamp,i
                    continue
                dt = (t-anchor[0])*1000
                ds = stamp-anchor[1]
                # Nearest grid point to the payload diagnoses phase; selecting
                # by host elapsed time independently tests timestamp prediction.
                n = round(ds/period)
                nh = round(dt/period)
                results.append([path.parent.name,r['event'],anchor[1],stamp,dt/1000,
                                ds-n*period,ds-round(ds/30)*30,ds-nh*period,nh-n])
    train = [r for r in results if r[0]=='S22B']
    durations = np.array([r[3]-r[2] for r in train])
    cycles = np.rint(durations/period)
    fitted = np.dot(cycles,durations)/np.dot(cycles,cycles)
    for r in results:
        duration = r[3]-r[2]
        r.extend([fitted, duration-round(duration/fitted)*fitted])
    held = np.array([[r[5],r[-1]] for r in results if r[0]!='S22B'])
    print('S22B fit; held-out N, median and max', fitted,len(held),
          np.median(abs(held),axis=0),np.max(abs(held),axis=0))
    with (root/'supervisor-phase.csv').open('w') as f:
        w = csv.writer(f)
        w.writerow('run kind anchor_ms payload_ms elapsed_host_s calibrated_phase_residual_ms nominal_phase_residual_ms host_prediction_residual_ms tick_index_error S22B_period_ms S22B_fit_phase_residual_ms'.split())
        w.writerows(results)
    fig, axes = plt.subplots(1,2,figsize=(9,3.8),layout='constrained')
    for name in sorted({r[0] for r in results}):
        rows=[r for r in results if r[0]==name]
        for ax,k in zip(axes,(5,7)):
            ax.scatter([r[4] for r in rows],[r[k] for r in rows],label=name,s=22)
    for ax,title in zip(axes,('Closest recorded clock tick','Tick predicted from host elapsed time')):
        ax.axhline(0,color='.5',lw=.8)
        ax.set(title=title,xlabel='Seconds since first callback',ylabel='Payload minus prediction (ms)')
    axes[1].legend(ncol=2,fontsize=7)
    fig.savefig(root/'supervisor-phase.png',dpi=165)
    a=np.array([r[4:] for r in results])
    print(len(results),len(set((r[0],r[2]) for r in results)),a[:,0].max(),
          np.median(abs(a[:,1:4]),axis=0),abs(a[:,1:4]).max(axis=0),sum(a[:,4]==0))


if __name__=='__main__':
    main()

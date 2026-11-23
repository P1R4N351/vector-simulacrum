"""Separate listener memory from new packet input around the docked grip's release."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from simulacrum.cube import MovementListener, engine_samples, packet

root = Path(__file__).resolve().parent


def main():
    groups = np.loadtxt(root/'cube.csv',delimiter=',',skiprows=1).reshape(-1,3,5)
    assert np.all(groups[:,:,1]==[0,1,2])
    times = groups[:,0,0]
    gaps = np.flatnonzero((times[:-1]>1.3) & (times[1:]<1.9) & (np.diff(times)>.15))
    restart = gaps.item()+1
    samples = np.stack([engine_samples(packet(0,g[:,2:].astype(int).tolist())) for g in groups])
    # Use the physically settled orientation for the quiet-packet alternative.
    quiet = samples[times>2.5]
    assert len(quiet)>0
    modes = ['Recorded', 'Hold last vector', 'Settled packets', 'Reset score', 'Reset filter', 'Reset listener']
    fig,ax=plt.subplots(figsize=(8,4),layout='constrained')
    rows = []
    for mode in modes:
        listener = MovementListener()
        points = []
        for i,(t,group) in enumerate(zip(times,samples)):
            if i == restart:
                assert not listener.moving
                if mode=='Reset score':
                    listener.score = 0
                elif mode=='Reset filter':
                    saved_score = listener.score
                    listener = MovementListener()
                    listener.score = saved_score
                elif mode=='Reset listener':
                    listener = MovementListener()
            if i>=restart:
                if mode=='Hold last vector':
                    group=np.repeat(samples[restart-1,-1][None,:],3,axis=0)
                elif mode=='Settled packets':
                    group=quiet[(i-restart)%len(quiet)]
            for xyz in group:
                listener.update(t,xyz)
                points.append((t,listener.score))
        tt,ss=np.asarray(points).T
        before=ss[tt<times[restart]][-1]
        after=tt>=times[restart]
        events=listener.events
        rows.append([mode,before,ss[after].max(),next((t for t,m in events if m),None),
                     next((t for t,m in events if not m),None)])
        ax.plot(tt[after],ss[after],'.-',ms=2,lw=1,label=mode)
        if mode=='Recorded':
            ax.plot(tt[~after],ss[~after],'.-',ms=2,color='C0')
    ax.axvspan(times[restart-1],times[restart],alpha=.12,color='grey',label='No received packets')
    ax.axhline(10000,color='grey',ls=':',lw=1)
    ax.set(xlim=(1.0,2.3),xlabel='Seconds from lift command (packet receipt)',ylabel='Movement listener score')
    ax.legend(fontsize=8,ncol=2)
    fig.savefig(root/'gap-memory.png',dpi=165)
    with (root/'gap-memory.csv').open('w') as f:
        w=csv.writer(f)
        w.writerow(['replay','score_before_gap','peak_after_gap','start_receipt_s','stop_receipt_s'])
        w.writerows(rows)


if __name__=='__main__':
    main()

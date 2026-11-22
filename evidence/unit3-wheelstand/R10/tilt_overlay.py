"""Compare cube tilt on the video clock with replay and specific force on the receipt clock."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

HERE=Path(__file__).resolve().parent


def main():
    with (HERE/'tilt-frames.csv').open() as f:
        frames=list(csv.DictReader(f))
    with (HERE/'actions.csv').open() as f:
        start=float(next(r['sec'] for r in csv.DictReader(f) if r['action']=='dock' and r['kind']=='begin'))
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True,layout='constrained')
    for view,marker in [('A','o'),('B','s')]:
        a=np.array([[float(r[k]) for k in ('dock_s','median_deg','threshold_p10_deg','threshold_p90_deg')]
                    for r in frames if r['view']==view])
        t,v,lo,hi=a.T
        axes[0].errorbar(t,v,yerr=[v-lo,hi-v],fmt=marker,ms=3,label=f'Camera {view}')
    a=np.array([[float(r[k]) for k in ('dock_s','median_deg')] for r in frames if r['view']=='A'])
    for order in ('original','reverse'):
        pred=np.genfromtxt(HERE/f'tilt-{order}.csv',names=True,delimiter=',')
        t,v=pred['dock_s'],pred['tilt_deg']
        axes[0].plot(t,v,lw=1,label=f'Sim · {order} box order')
        at_frames=np.interp(a[:,0],t,v)
        print(order,'peak time/deg',t[v.argmax()],v.max(),'sampled peak',at_frames.max(),
              'frames >30',sum(at_frames>30))
    cube=np.genfromtxt(HERE/'cube.csv.gz',names=True,delimiter=',')
    t=cube['sec']-start
    g=np.column_stack([cube[k] for k in 'xyz'])/8192
    norm=np.linalg.norm(g,axis=1)
    baseline=g[(t>-1)&(t<0)].mean(axis=0)
    theta=np.degrees(np.arctan2(np.linalg.norm(np.cross(g,baseline),axis=1),g@baseline))
    sel=(t>=4)&(t<=6.5)
    axes[1].scatter(t[sel],theta[sel],s=9,c=abs(norm[sel]-1),cmap='viridis',vmin=0,vmax=.8)
    for tt in t[1:][np.diff(cube['tap'])!=0]:
        if 4<tt<6.5:
            axes[1].axvline(tt,color='0.6',lw=1)
    cb=fig.colorbar(axes[1].collections[0],ax=axes[1])
    cb.set_label('Distance from 1 g (g)')
    axes[0].legend(ncol=2)
    axes[0].set_ylabel('Top-plane tilt (°)')
    axes[1].set_ylabel('Specific-force direction\nfrom baseline (°)')
    axes[1].set_xlabel('Seconds from dock command')
    axes[1].set_xlim(4,6.5)
    print('largest apparent angle',theta[sel].max(),'at',t[sel][theta[sel].argmax()],'norm',norm[sel][theta[sel].argmax()])
    fig.savefig(HERE/'tilt-overlay.png',dpi=150)


if __name__=='__main__':
    main()

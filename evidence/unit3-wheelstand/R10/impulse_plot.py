"""Compare simulated impulses and measured interval yaw rates on the dock clock."""
import csv
import matplotlib.pyplot as plt
import numpy as np
from impact_wrench import HERE


def main():
    with (HERE/'impact-path-camera.csv').open() as f:
        optical=[r for r in csv.DictReader(f) if r['view']=='A']
    with (HERE/'impact-path-candidates.csv').open() as f:
        candidates=list(csv.DictReader(f))
    fig,axes=plt.subplots(3,2,figsize=(11,7),sharex=True,sharey='row',layout='constrained')
    for col,view in enumerate(('A','B')):
        a=np.genfromtxt(HERE/f'impact-impulse-{view}.csv.gz',names=True,delimiter=',')
        t=a['dock_s']
        for partner in ('fork','shelf'):
            axes[0,col].plot(t,a[partner+'_jz_Nms']*1000,label=partner)
        axes[0,col].plot(t,a['cube_Lz_Nms']*1000,color='black',lw=1,label='Cube angular momentum')
        axes[0,col].set_title(f'Pose from camera {view} · orientation 2')
        for partner in ('fork','shelf'):
            idx=np.arange(0,len(t),10)
            avg=np.diff(a[partner+'_vertical_Ns'][idx])/np.diff(t[idx])
            axes[1,col].plot((t[idx][1:]+t[idx][:-1])/2,avg,label=partner)
        axes[1,col].axhline(.05*9.81,color='gray',ls=':',label='Cube weight')
        sim=[r for r in candidates if r['view']==view and r['layout']=='2']
        for rows,key,label in ((optical,'fit_ccw_deg','Camera A'),(sim,'ccw_deg','Replay')):
            times=np.array([float(r['dock_s']) for r in rows])
            angle=np.array([float(r[key]) for r in rows])
            axes[2,col].stairs(np.diff(angle)/np.diff(times),times,baseline=None,label=label)
        axes[2,col].set_xlim(4.16,4.67)
    for ax in axes[:,0]: ax.legend(fontsize=8)
    axes[0,0].set_ylabel('Angular impulse\n(mN m s, CCW positive)')
    axes[1,0].set_ylabel('Vertical force on cube\n(N, up positive)')
    axes[2,0].set_ylabel('Mean yaw rate between\nvideo frames (°/s)')
    fig.supxlabel('Seconds from dock command')
    fig.savefig(HERE/'impact-impulse.png',dpi=150)


if __name__=='__main__':
    main()

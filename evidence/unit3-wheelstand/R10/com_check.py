"""Test fixed 3 mm center offsets on the same physical cube strike."""
import csv
from collections import defaultdict
import matplotlib.pyplot as plt
import numpy as np
from pose_grid import HERE, pose, read, score


def main():
    camera={(r['view'],int(r['frame'])):pose(r,'fit_') for r in read('impact-path-camera.csv')}
    baseline={(r['view'],int(r['frame'])):pose(r) for r in read('inertia-path.csv')
              if float(r['inertia_multiplier'])==1}
    groups=defaultdict(dict)
    for r in read('com-path.csv'):
        center=tuple(int(r['com_'+axis+'_mm']) for axis in 'xyz')
        view,frame=r['view'],int(r['frame'])
        groups[view,center][frame]=pose(r)
        if not any(center):
            np.testing.assert_allclose(pose(r),baseline[view,frame],atol=1e-6)
    frames=range(75,85)
    train=[(view,f) for view in 'AB' for f in (76,77)]
    later=[('A',f) for f in range(78,85)]
    def metrics(key,window):
        return score(np.array([groups[key][f] for _,f in window]),
                     np.array([camera[v,f] for v,f in window]))
    fig,axes=plt.subplots(2,2,figsize=(10,6),layout='constrained',sharex=True)
    with (HERE/'com-scores.csv').open('w') as stream:
        w=csv.writer(stream,lineterminator='\n')
        w.writerow(['registration','com_x_mm','com_y_mm','com_z_mm','selected','early_corner_rms_mm',
                    'yaw_4_583_deg','final_yaw_deg','later_corner_rms_mm','later_xy_rms_mm','later_yaw_rms_deg'])
        for col,view in enumerate('AB'):
            keys=[k for k in groups if k[0]==view]
            best=min(keys,key=lambda k:metrics(k,train)[0])
            for k in keys:
                s=groups[k]
                w.writerow([view,*k[1],int(k==best),metrics(k,train)[0],s[79][2],s[101][2],*metrics(k,later)])
            for ax,j in zip(axes[:,col],(1,2)):
                for k in sorted(keys,key=lambda k:(not any(k[1]),k==best)):
                    chosen=k==best
                    label=f'Selected {k[1]} mm' if chosen else 'Centered' if not any(k[1]) else None
                    ax.plot([f/12-2 for f in frames],[groups[k][f][j] for f in frames],
                            color='tab:blue' if chosen else 'tab:orange' if label else '.75',
                            lw=2 if label else 1,label=label)
                ax.plot([f/12-2 for f in frames],[camera['A',f][j] for f in frames],'ko-',ms=3,label='Camera A')
            axes[0,col].set_title('Initial registration '+view)
            axes[0,col].legend(fontsize=8)
            axes[1,col].set_xlabel('Seconds from dock command')
    axes[0,0].set_ylabel('Left displacement (mm)')
    axes[1,0].set_ylabel('Counterclockwise yaw (°)')
    fig.savefig(HERE/'com-transfer.png',dpi=160)


if __name__=='__main__':
    main()

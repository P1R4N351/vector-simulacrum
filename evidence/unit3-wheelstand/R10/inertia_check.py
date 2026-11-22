"""Score isotropic mass redistribution at the physical camera frame times."""
import csv
from collections import defaultdict
import matplotlib.pyplot as plt
import numpy as np
from pose_grid import HERE, pose, read, score


def main():
    camera={int(r['frame']):pose(r,'fit_') for r in read('impact-path-camera.csv') if r['view']=='A'}
    baseline={(r['view'],int(r['frame'])):pose(r) for r in read('pose-grid.csv')
              if int(r['offset_forward_mm'])==int(r['offset_left_mm'])==0}
    groups=defaultdict(dict)
    for r in read('inertia-path.csv'):
        key=(r['view'],float(r['inertia_multiplier']))
        frame=int(r['frame'])
        groups[key][frame]=pose(r)
        if key[1]==1:
            np.testing.assert_allclose(pose(r),baseline[key[0],frame],atol=1e-6)
    frames=np.arange(75,85)
    obs=np.array([camera[f] for f in frames])
    fig,axes=plt.subplots(2,2,figsize=(9,6),sharex=True,layout='constrained')
    with (HERE/'inertia-scores.csv').open('w') as stream:
        w=csv.writer(stream,lineterminator='\n')
        w.writerow(['registration','inertia_multiplier','yaw_4_583_deg','final_yaw_deg',
                    'moving_corner_rmse_mm','moving_xy_rmse_mm','moving_yaw_rmse_deg'])
        for col,view in enumerate('AB'):
            for ax,k in zip(axes[:,col],(1,2)):
                ax.plot(frames/12-2,obs[:,k],'ko-',ms=3,label='Measured · camera A')
            for factor in sorted(k[1] for k in groups if k[0]==view):
                series=groups[view,factor]
                pred=np.array([series[f] for f in frames])
                for ax,k in zip(axes[:,col],(1,2)):
                    ax.plot(frames/12-2,pred[:,k],label=f'{factor:.2f} × inertia')
                w.writerow([view,factor,series[79][2],series[101][2],*score(pred[3:],obs[3:])])
            axes[0,col].set_title('Initial registration '+view)
            axes[1,col].set_xlabel('Seconds from dock command')
    axes[0,0].set_ylabel('Left displacement (mm)')
    axes[1,0].set_ylabel('Counterclockwise yaw (°)')
    axes[0,1].legend(fontsize=8)
    fig.savefig(HERE/'inertia.png',dpi=150)


if __name__=='__main__':
    main()

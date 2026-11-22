"""Compare fixed and actuated shoulder trajectories on matched camera frames."""
import csv

import matplotlib.pyplot as plt
import numpy as np
from pose_grid import HERE, pose, read, score


def main():
    camera={int(r['frame']):pose(r,'fit_') for r in read('impact-path-camera.csv') if r['view']=='A'}
    baseline={(r['view'],int(r['frame'])):pose(r) for r in read('inertia-path.csv')
              if float(r['inertia_multiplier'])==1}
    rows=read('fixed-path.csv')
    frames=range(74,83)
    obs=np.array([camera[f] for f in frames])
    times=np.arange(74,83)/12-2
    fig,axes=plt.subplots(2,2,sharex=True,figsize=(9,6),layout='constrained')
    with (HERE/'fixed-scores.csv').open('w') as stream:
        w=csv.writer(stream,lineterminator='\n')
        w.writerow(['view','fixed','yaw_4_583_deg','late_corner_rms_mm','late_xy_rms_mm','late_yaw_rms_deg'])
        for col,view in enumerate('AB'):
            for fixed in (0,1):
                path={int(r['frame']):pose(r) for r in rows if r['view']==view and int(r['fixed_shoulder'])==fixed}
                pred=np.array([path[f] for f in frames])
                if not fixed:
                    np.testing.assert_allclose(pred,[baseline[view,f] for f in frames],atol=1e-6)
                w.writerow([view,fixed,path[79][2],*score(pred[4:],obs[4:])])
                for ax,j in zip(axes[:,col],(1,2)):
                    ax.plot(times,pred[:,j],'.-',label='Shoulder fixed' if fixed else 'Actuated')
            for ax,j in zip(axes[:,col],(1,2)):
                ax.plot(times,obs[:,j],'ko-',ms=3,label='Camera A')
            axes[0,col].set_title('Initial registration '+view)
            axes[1,col].set_xlabel('Seconds from dock command')
        axes[0,0].set_ylabel('Left displacement (mm)')
        axes[1,0].set_ylabel('Counterclockwise yaw (°)')
        axes[1,0].legend()
    fig.savefig(HERE/'fixed-shoulder.png',dpi=150)


if __name__=='__main__':
    main()

"""Select an initial translation before scoring the remainder of the camera trace."""
from collections import defaultdict
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

HERE=Path(__file__).resolve().parent


def read(name):
    with (HERE/name).open() as stream:
        return list(csv.DictReader(stream))


def pose(row,prefix=''):
    return np.array([float(row[prefix+key]) for key in ('forward_mm','left_mm','ccw_deg')])


def score(pred,ref):
    # RMS displacement of the four corresponding corners of a 23 mm square.
    # Their offsets sum to zero, so translation and rotation separate exactly.
    delta=pred-ref
    radius=23/np.sqrt(2)
    xy_sq=(delta[:,:2]**2).sum(axis=1)
    turn_mm=2*radius*np.sin(np.radians(delta[:,2])/2)
    return np.sqrt(np.mean(xy_sq+turn_mm**2)),np.sqrt(xy_sq.mean()),np.sqrt(np.mean(delta[:,2]**2))


def main():
    obs={(r['view'],int(r['frame'])):pose(r,'fit_') for r in read('impact-path-camera.csv')}
    grid=defaultdict(dict)
    for r in read('pose-grid.csv'):
        grid[r['view'],int(r['offset_forward_mm']),int(r['offset_left_mm'])][int(r['frame'])]=pose(r)
    fit=[(view,f) for view in 'AB' for f in (76,77)]
    holdout=[('A',f) for f in range(78,102)]
    motion=[('A',f) for f in range(78,85)]
    def compare(p,keys):
        return score(np.array([p[f] for _,f in keys]),np.array([obs[k] for k in keys]))
    winner=min(grid,key=lambda k:compare(grid[k],fit)[0])
    print('Selected',winner)
    with (HERE/'pose-grid-scores.csv').open('w') as stream:
        w=csv.writer(stream,lineterminator='\n')
        w.writerow(['view','offset_forward_mm','offset_left_mm','selected','fit_corner_rmse_mm',
                    'holdout_corner_rmse_mm','holdout_xy_rmse_mm','holdout_yaw_rmse_deg',
                    'moving_corner_rmse_mm','moving_xy_rmse_mm','moving_yaw_rmse_deg'])
        for k,p in grid.items():
            result=[*k,int(k==winner),compare(p,fit)[0],*compare(p,holdout),*compare(p,motion)]
            w.writerow(result)
            if k==winner or k[1:]==(0,0):
                print(result,'at frame 79',p[79],'final',p[101])
    fig,axes=plt.subplots(2,2,figsize=(10,7),layout='constrained')
    for col,keys in enumerate((fit,holdout)):
        for v,marker in [('A','o'),('B','s')]:
            fs=[f for name,f in keys if name==v]
            if not fs:continue
            pts=np.array([obs[v,f] for f in fs])
            axes[0,col].plot(pts[:,0],pts[:,1],marker,ms=4,label='Camera '+v)
            axes[1,col].plot(np.array(fs)/12-2,pts[:,2],marker,ms=4,label='Camera '+v)
        for k in sorted(grid,key=lambda k:k==winner):
            style='-' if k==winner else ':'
            shown=k==winner or k[1:]==(0,0)
            attrs={} if shown else dict(color='gray',alpha=.22,linewidth=.8)
            frames=sorted({f for _,f in keys})
            points=np.array([grid[k][f] for f in frames])
            label=(f'{k[0]} ({k[1]:+d}, {k[2]:+d}) mm' + (' · selected' if k==winner else '')) if shown else None
            axes[0,col].plot(points[:,0],points[:,1],style,label=label,**attrs)
            axes[1,col].plot(np.array(frames)/12-2,points[:,2],style,label=label,**attrs)
        axes[0,col].set_aspect('equal',adjustable='datalim')
        axes[0,col].set_xlabel('Forward displacement (mm)')
        axes[0,col].set_title(('First two moving frames','Subsequent unused frames')[col])
        axes[1,col].set_xlabel('Seconds from dock command')
    axes[0,0].set_ylabel('Left displacement (mm)')
    axes[1,0].set_ylabel('Counterclockwise yaw (°)')
    axes[1,0].legend(fontsize=8)
    fig.savefig(HERE/'pose-grid.png',dpi=150)


if __name__=='__main__':
    main()

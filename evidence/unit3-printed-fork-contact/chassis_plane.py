"""Intersect tracked crown rays with horizontal planes and replay the inferred shift."""
import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from evaluate import here,states,run
from optical_clearance import quads,rows,marker

a=np.genfromtxt(here/'chassis-track.csv',names=True,delimiter=',')
principal=np.array([912.9463,541.1996])
focal=473.11
K=np.diag([focal,focal,1.])
K[:2,2]=principal
def undistort(q):
    d=q+[420,485]-principal
    return principal+d/(1-1.499*np.sum((d/960)**2,axis=1)[:,None])
ok,rv,tv=cv2.solvePnP(marker,undistort(np.median(quads[rows['sec']<0],axis=0)),K,None)
assert ok
inv=Rotation.from_rotvec(rv.ravel()).inv()
camera=inv.apply(-tv.ravel())
idx=states['trial']==1
original=np.column_stack([states[col][idx] for col in ('x_mm','y_mm')])
t=states['sec'][idx]-1.5
scores=[]
fig,axes=plt.subplots(2,1,sharex=True,figsize=(8,6),layout='constrained')
for patch,center in [('vents',[377.5,262]),('seam',[378,246.5])]:
    valid=a[patch+'_corr']>.75
    stamp=a['sec'][valid]
    uv=center+np.column_stack([a[patch+suffix][valid] for suffix in ('_dx','_dy')])
    rays=inv.apply(np.c_[(undistort(uv)-principal)/focal,np.ones(len(uv))])
    odom=np.column_stack([np.interp(stamp,t,col) for col in original.T])
    for height in (60,70,80):
        pos=camera+rays*((height-22.15-camera[2])/rays[:,2,None])
        displacement=pos[:,:2]-np.median(pos[stamp<0,:2],axis=0)
        actual=odom-np.median(odom[stamp<0],axis=0)
        # Use approach only to establish horizontal heading. Preserve metric scale.
        train=(stamp>0)&(stamp<3)
        dot=np.sum(displacement[train]*actual[train])
        cross=np.sum(displacement[train,0]*actual[train,1]-displacement[train,1]*actual[train,0])
        yaw=np.arctan2(cross,dot)
        matrix=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
        residual=displacement@matrix.T-actual
        for j,ax in enumerate(axes):
            ax.plot(stamp,residual[:,j],label=f'{patch}, {height} mm')
        raised=np.argmin(abs(stamp-5.498))
        hold=(stamp>=8)&(stamp<=10)
        for dims in ('lateral','both'):
            for j,col in enumerate(('x_mm','y_mm')):
                states[col][idx]=original[:,j]
                if j==1 or dims=='both':
                    states[col][idx]+=np.interp(t,stamp,residual[:,j])
            result,_=run((1,'stl','overhead','weld',0,0,0),rails=True)
            scores.append((patch,height,dims,np.degrees(yaw),*residual[raised],
                           *np.sqrt(np.mean(residual[hold]**2,axis=0)),result[-1],-result[9],result[-3]))
for j,col in enumerate(('x_mm','y_mm')): states[col][idx]=original[:,j]
for ax,label in zip(axes,('Forward','Leftward')):
    ax.set_ylabel(label+' residual (mm)')
    ax.axvspan(0,3,color='gray',alpha=.12)
axes[1].set_xlabel('Seconds after approach')
axes[0].legend(ncol=3,fontsize=8)
fig.savefig(here/'chassis-plane.png',dpi=160)
with (here/'chassis-plane.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow('patch height_mm replay fitted_heading_deg raised_forward_mm raised_left_mm retreat_forward_rms_mm retreat_left_rms_mm tilt_deg right_mm endpoint_error_mm'.split())
    w.writerows(scores)

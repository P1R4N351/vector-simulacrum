"""Joint rigid-body fit to opposite visible hub faces in the two corrected cameras."""
import csv
import gzip
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

here=Path(__file__).resolve().parent
K=np.array([[465,0,499.5],[0,465,185.5],[0,0,1.]])
world=np.array([[-23,0,71.5],[23,0,71.5],[23,0,25.5],[-23,0,25.5],[0,-94,0]],float)
picks=[[[79,180],[166,162],[187,253],[104,292],[345,330]],[[855,90],[951,100],[921,218],[833,187],[668,273]]]
with gzip.open(here/'states.csv.gz','rt') as f:
    sdk=np.genfromtxt(f,delimiter=',',names=True,dtype=None,encoding='utf8')
i=np.flatnonzero(sdk['status']&4096)[0]
st=sdk['receipt_s']-sdk['receipt_s'][i]
pitch=np.degrees(sdk['pitch_rad'])
j=np.flatnonzero((pitch[:i]>-5)&(pitch[1:i+1]<=-5))[-1]+1
peak=st[j+np.argmin(pitch[j:i+1])]
times=np.unique(np.r_[peak,np.arange(-1,0,.04),0.])
tracks=[]
for f in ('opposed-hubs.csv','dense-hubs.csv'):
    r=np.genfromtxt(here/f,delimiter=',',names=True,dtype=None,encoding='utf8')
    r=r[r['direction']=='backward']
    assert r['t_s'].min()<peak and r['t_s'].max()>0
    tracks.append(np.stack([np.stack([np.interp(times,r['t_s'],r[hub+'_'+axis]-r['dock_'+axis]+np.median(r['dock_'+axis])) for axis in ('x','y')],axis=1) for hub in ('front','rear')],axis=1))
obs=np.concatenate(tracks,axis=1)
output=[]
validation=[]
fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
for count in (4,5):
    cameras=[]
    for marker in picks:
        pix=np.array(marker,float)
        ok,rv,tv=cv2.solvePnP(world[:count],pix[:count],K,None,flags=cv2.SOLVEPNP_SQPNP)
        assert ok
        rv,tv=cv2.solvePnPRefineLM(world[:count],pix[:count],K,None,rv,tv)
        R=cv2.Rodrigues(rv)[0]
        cameras.append((R,tv.ravel(),np.sign((-R.T@tv)[0,0])))
    for side in (27,30,33):
        hubsets=[np.array([[sign*side,-24,1.875],[sign*side,24,-1.875]]) for R,tv,sign in cameras]
        def residual(params,measurement):
            rot=Rotation.from_rotvec(params[3:])
            proj=[]
            for hubs,(R,tv,_) in zip(hubsets,cameras):
                coords=(rot.apply(hubs)+params[:3])@R.T+tv
                uv=coords@K.T
                proj.extend(uv[:,:2]/uv[:,2:])
            return (np.array(proj)-measurement).ravel()
        initial=np.array([0,-80,12,0,0,0])
        poses=[]
        for time,measurement in zip(times,obs):
            fit=least_squares(residual,initial,args=(measurement,),max_nfev=90)
            assert fit.success
            initial=fit.x
            rot=Rotation.from_rotvec(initial[3:])
            axis=rot.apply([0.,1.,0.])
            yaw=np.degrees(np.arctan2(-axis[0],axis[1]))
            poses.append([time,*initial[:3],yaw,np.sqrt(np.mean(fit.fun**2))])
        p=np.array(poses)
        if side==30:
            for idx in [0,len(times)-1]:
                for hub in range(4):
                    mask=np.arange(8)//2!=hub
                    held=least_squares(lambda x: residual(x,obs[idx])[mask],initial,max_nfev=150)
                    assert held.success
                    err=np.linalg.norm(residual(held.x,obs[idx])[~mask])
                    validation.append([count,times[idx],hub,err])
        final=np.argmin(abs(times))
        delta=p[final]-p[0]
        print(count,side,'inward/lateral/yaw, max pixel RMS',np.round([delta[2],delta[1],delta[4],p[:,5].max()],2))
        output.extend([[count,side,*row] for row in p])
        if side==30:
            axes[0].plot(times,p[:,2]-p[0,2],label=f'{count} dock anchors')
            axes[1].plot(times,p[:,4]-p[0,4],label=f'{count} dock anchors')
sim=np.genfromtxt(here/'registered-center.csv',names=True,delimiter=',',dtype=None,encoding='utf8')
for variant,label in [('canonical','Canonical sim'),('driven_rear_envelope','Driven rear support')]:
    rows=sim[sim['variant']==variant]
    axes[0].plot(rows['t_s'],rows['inward_mm'],'--',label=label)
    axes[1].plot(rows['t_s'],rows['yaw_deg'],'--',label=label)
hw=(st>=peak)&(st<0)
axes[1].plot(st[hw],np.degrees(sdk['yaw_rad'][hw]-sdk['yaw_rad'][hw][0]),'k.',label='SDK (before reset)')
for a in axes:a.set_xlabel('Seconds from physical first charger flag');a.legend(fontsize=8)
axes[0].set_ylabel('Hub midpoint travel inward (mm)')
axes[1].set_ylabel('Change in chassis yaw (degrees)')
fig.savefig(here/'opposed-pose.png',dpi=165)
with (here/'opposed-pose.csv').open('w') as f:
    w=csv.writer(f);w.writerow('anchors side_mm t_s lateral_mm inward_mm z_mm yaw_deg rms_px'.split());w.writerows(output)

with (here/'opposed-withheld.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['anchors','t_s','omitted_hub_Afront0_Arear1_Bfront2_Brear3','prediction_error_px']);w.writerows(validation)

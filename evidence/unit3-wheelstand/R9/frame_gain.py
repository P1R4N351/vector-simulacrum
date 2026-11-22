"""Check cube-frame sign and held-out sideways gravity response in roll recordings."""
import csv
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

here = Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent/'R8'))
from tread_speed import load_rows, run

def accel(path, names=('x','y','z')):
    rows=np.genfromtxt(path,delimiter=',',names=True)
    return rows,np.column_stack([rows[k] for k in names])/8192

prior=here.parents[1]/'unit3-cube-accelerometer'
_,rest=accel(prior/'resting.csv',('x_counts','y_counts','z_counts'))
gain=np.linalg.norm(rest.mean(axis=0))
z_true=np.sqrt(1-rest.mean(axis=0)[:2]@rest.mean(axis=0)[:2])
z_gain=rest[:,2].mean()/z_true
c,v=accel(here/'cube.csv.gz')
start=next(float(r['sec']) for r in load_rows('actions.csv',here) if r['action']=='roll cube')
_,settled=accel(here.parent/'R8/settled.csv.gz')
windows={'Upright fit':rest,'After roll':settled,'Next approach':v[(c['sec']-start>=-1)&(c['sec']<start)]}
scores={}
with (here/'side-gravity.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('window','samples','candidate','measured_mean_norm_g','inferred_gravity_error_mg'))
    for label,scale in [('Nominal',np.ones(3)),('Z gain',[1,1,z_gain]),('Common gain',[gain]*3)]:
        scores[label]=[]
        for name,values in windows.items():
            actual=values.mean(axis=0)
            error=1000*(np.linalg.norm(actual/scale)-1)
            scores[label].append(error)
            w.writerow((name,len(values),label,np.linalg.norm(actual),error))
# Mean directions use SDK poses independently of measured gravity.
rows=load_rows('roll-poses.csv',here)
poses=[r for r in rows if r['source']=='robot_observed_object']
mean=windows['Next approach'].mean(axis=0)
obs=mean/np.linalg.norm(mean)
with (here/'frame-sign.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('s','sensor_yaw_deg','predicted_gx','predicted_gy','predicted_gz','angle_to_measured_deg'))
    for r in poses:
        q=Rotation.from_quat([float(r[k]) for k in ('qw','qx','qy','qz')],scalar_first=True)
        for yaw in (0,180):
            prediction=(q*Rotation.from_euler('z',yaw,degrees=True)).inv().apply([0,0,1])
            w.writerow((r['sec'],yaw,*prediction,np.degrees(np.arccos(np.clip(prediction@obs,-1,1)))))
fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
for label,errors in scores.items():
    axes[0].plot(list(windows),errors,'o-',label=label)
axes[0].axhline(0,c='.5',lw=.8)
axes[0].set(ylabel='Resting gravity error (mg)');axes[0].legend()
raw=here.parent/'R8'
cr,physical=accel(raw/'cube.csv.gz')
zero=next(float(r['sec']) for r in load_rows('actions.csv',raw) if r['action']=='roll cube')
axes[1].plot(cr['sec']-zero,physical[:,1],'.',ms=2,label='Measured Y')
with (here/'dynamic-frame.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n');w.writerow(('yaw_deg','isotropic_gain','last_filtered_axis','tap_changes_after_2_5s','peak_packet_g_after_2_5s','common_window_xyz_rmse_g'))
    for yaw,g in [(0,1.),(180,1.),(180,gain)]:
        t,tilt,hit,speed,sensor=run(sys.argv[1],.00025,True,sensor_overrides={'cube_chip_yaw_deg':yaw,'cube_accel_scale':[g]*3})
        ts=[];acc=[]
        for stamp,b in sensor.messages:
            ts.append(stamp)
            acc.append(np.frombuffer(b[2:],'<i2').reshape(3,3)/8192)
        ts=np.array(ts);acc=np.array(acc)
        # Compare packet means at reception; this is sensitive to timing during impacts.
        pt=(cr['sec']-zero).reshape(-1,3)[:,0];pv=physical.reshape(-1,3,3).mean(axis=1)
        select=(pt>=2.5)&(pt<=ts[-1])
        pred=np.column_stack([np.interp(pt[select],ts,a) for a in acc.mean(axis=1).T])
        w.writerow((yaw,g,sensor.up_axis.axis,sum(tap>2.5 for tap in sensor.taps.events),
                    np.linalg.norm(acc[ts>2.5],axis=2).max(),np.sqrt(np.mean(np.sum((pred-pv[select])**2,axis=1)))))
        if g==1:axes[1].plot(ts,acc.mean(axis=1)[:,1],lw=1,label=f'{yaw}° sensor yaw')
axes[1].set(xlabel='Seconds after roll',ylabel='Cube Y (g)',xlim=(3,5.25));axes[1].legend()
fig.savefig(here/'frame-gain.png',dpi=150)

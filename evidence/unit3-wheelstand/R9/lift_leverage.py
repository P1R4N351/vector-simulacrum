"""Measure the contact wrench and quasi-static tipping demand for the recorded roll."""
import csv
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from yaw_transfer import cube_rotation,here,load_rows,run

def test(case):
    name,limit,path,supported=case
    directory=here.parent/name
    delta=next(float(r['grid_yaw_minus_sdk_deg']) for r in load_rows('optical-registration.csv',here)
               if r['calibration']=='Dock and first cube' and r['view']==('Before roll' if name=='R8' else 'Next approach'))
    rotation=cube_rotation(directory,delta)
    # Horizontal square axes; the X axis stays horizontal in both settled poses.
    right=rotation.apply([1,0,0])[:2]
    right/=np.linalg.norm(right)
    basis=np.array([right,[-right[1],right[0]]])
    stream=[]
    def record(m,d):
        if d.time<3 or d.time>5.21:return
        cid=m.body('cube').id
        mass=m.body_mass[cid]
        origin=d.xipos[cid].copy()
        origin[2]=0
        force=mass*m.opt.gravity.copy()
        torque=np.cross(d.xipos[cid]-origin,force)
        hand=np.zeros(3)
        for i,ct in enumerate(d.contact):
            b1,b2=m.geom_bodyid[ct.geom]
            if cid not in (b1,b2):continue
            other=b1 if cid==b2 else b2
            if other==0:continue
            w=np.zeros(6)
            mj.mj_contactForce(m,d,i,w)
            signed=(w.reshape(2,3)@ct.frame.reshape(3,3))*(1 if cid==b2 else -1)
            force+=signed[0]
            torque+=np.cross(ct.pos-origin,signed[0])+signed[1]
            if other==m.body('fork').id:hand+=signed[0]
        down=-force[2]
        cop=basis@np.array([torque[1],-torque[0]])/down*1000 if down>1e-6 else np.full(2,np.nan)
        ctrl=d.actuator('lift').ctrl[0]
        angle=d.joint('lift_joint').qpos[0]
        stream.append((d.time,*hand,1000*d.actuator('lift').force[0],
                       66*(np.sin(angle)-np.sin(ctrl)),*cop))
    rest=np.genfromtxt(here.parents[1]/'unit3-cube-accelerometer/resting.csv',names=True,delimiter=',')
    gain=np.linalg.norm([rest[k+'_counts'].mean() for k in 'xyz'])/8192
    t,tilt,_,_,sensor=run(path,.00025,True,directory,rotation,{'servo_torque_max_mNm':limit,
            'cube_chip_yaw_deg':180,'cube_accel_scale':[gain]*3},articulated=True,observer=record,shelf_support=supported)
    a=np.asarray(stream)
    active=np.linalg.norm(a[:,1:4],axis=1)>.05
    packets=np.vstack([np.frombuffer(b[2:],'<i2').reshape(3,3) for ts,b in sensor.messages if ts>2.5])/8192
    scores=[name,limit,active.sum()*.00025, np.sum(active & (abs(a[:,4])>=limit-.01))*.00025,
            -min(a[:,3]),max(np.linalg.norm(a[:,1:3],axis=1)),max(abs(a[:,5])),
            np.max(abs(a[active,6:])) if any(active) else None,max(tilt),max(np.linalg.norm(packets,axis=1)),
            sum(ts>2.5 for ts in sensor.taps.events)]
    above=active & (np.max(abs(a[:,6:]),axis=1)>22.15)
    edges=np.diff(np.r_[False,above,False].astype(int))
    lengths=np.flatnonzero(edges==-1)-np.flatnonzero(edges==1)
    scores.extend([above.sum()*.25,max(lengths,default=0)*.25,t[tilt.argmax()],
                   a[active,0][0] if any(active) else None,supported])
    return scores,a

if __name__=='__main__':
    cases=[(name,limit,sys.argv[1],False) for name in ('R8','R9') for limit in (10,30,80)]
    cases.extend((name,30,sys.argv[1],True) for name in ('R8','R9'))
    with ProcessPoolExecutor(3) as pool:
        results=list(pool.map(test,cases))
    fig,axes=plt.subplots(3,1,figsize=(9,7),sharex=True,layout='constrained')
    with (here/'lift-leverage.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(('run','torque_limit_mNm','fork_loaded_s','saturated_while_loaded_s','max_downward_fork_N',
                    'max_horizontal_fork_N','peak_height_error_mm','max_required_support_offset_mm','peak_tilt_deg','peak_packet_g','taps','outside_support_ms','longest_outside_support_ms','peak_tilt_time_s','first_loaded_time_s','shelf_support'))
        for scores,a in results:
            w.writerow(scores)
            if scores[0]!='R8':continue
            label=f'{scores[1]} mN·m' + (' shelf-supported' if scores[-1] else '')
            axes[0].plot(a[:,0],-a[:,3],label=label)
            axes[1].plot(a[:,0],a[:,5])
            axes[2].plot(a[:,0],np.max(abs(a[:,6:]),axis=1))
    axes[0].set(ylabel='Downward fork force (N)',title='Loaded fork and support demand during the physical roll replay')
    axes[0].legend()
    axes[1].set_ylabel('Height error (mm)')
    axes[2].axhline(22.15,color='.3',ls='--',label='Cube half-width')
    axes[2].set(xlabel='Seconds after roll command',ylabel='Required support offset (mm)',xlim=(3.8,5.2))
    axes[2].legend()
    fig.savefig(here/'lift-leverage.png',dpi=150)

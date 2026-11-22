"""Carry independently measured cube yaw into torque-limited roll replays."""
import csv
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent/'R8'))
from tread_speed import run,load_rows

def cube_rotation(directory,delta):
    name=directory.name
    sighting=next(r for r in load_rows('roll-poses.csv',directory)
                  if r['source']=='robot_observed_object' and float(r['sec'])<0)
    if name=='R8':
        initial=Rotation.from_euler('z',float(sighting['yaw_rad']))
    else:
        raw=Rotation.from_quat([float(sighting[k]) for k in ('qw','qx','qy','qz')],scalar_first=True)
        up=raw.inv().apply([0,0,1])
        axis=np.eye(3)[np.argmax(abs(up))]*np.sign(up[np.argmax(abs(up))])
        correction,_=Rotation.align_vectors([[0,0,1]],[raw.apply(axis)])
        initial=correction*raw
    registered=Rotation.from_euler('z',delta,degrees=True)*initial
    if delta==0:
        registered=None if name=="R8" else initial
    return registered

def replay(case):
    fixture,name,label,delta=case
    directory=here.parent/name
    registered=cube_rotation(directory,delta)
    rest=np.genfromtxt(here.parents[1]/'unit3-cube-accelerometer/resting.csv',names=True,delimiter=',')
    gain=np.linalg.norm([rest[k+'_counts'].mean() for k in 'xyz'])/8192
    xy=None
    if label=='Optical + range':
        fit=next(r for r in load_rows('range-check.csv',here) if r['run']==name and r['initial_xy']=='Range fit and optical yaw')
        xy=np.round([float(fit['x_mm']),float(fit['y_mm'])],3)
    t,tilt,hit,_,sensor=run(fixture,.00025,True,directory,registered,
                           {'cube_chip_yaw_deg':180,'cube_accel_scale':[gain]*3},articulated=True,initial_xy_mm=xy)
    cutoff=t[-1]
    payloads=[payload for ts,payload in sensor.messages if 2.5<ts<=cutoff]
    signals=np.vstack([np.frombuffer(b[2:],'<i2').reshape(3,3) for b in payloads])/8192
    return [name,label,delta,t[hit][0] if any(hit) else None,max(tilt),tilt[-1],
            np.linalg.norm(signals,axis=1).max(),sum(2.5<ts<=cutoff for ts in sensor.taps.events)]

if __name__=='__main__':
    cases=[]
    for name,view,low,high in [('R8','Before roll',.992,13.584),('R9','Next approach',-6.334,4.497)]:
        measured=next(float(r['grid_yaw_minus_sdk_deg']) for r in load_rows('optical-registration.csv',here)
                      if r['calibration']=='Dock and first cube' and r['view']==view)
        for label,delta in [('SDK',0),('Low',low),('Optical',measured),('High',high),('Optical + range',measured)]:
            cases.append((sys.argv[1],name,label,delta))
    with ProcessPoolExecutor(4) as pool:
        rows=list(pool.map(replay,cases))
    with (here/'yaw-transfer.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(('run','initialization','yaw_change_deg','first_tread_contact_s','peak_tilt_deg','last_tilt_deg','peak_packet_g','counter_changes'))
        w.writerows(rows)
    fig,axes=plt.subplots(1,2,figsize=(9,3.9),layout='constrained')
    for ax,name,observed in zip(axes,('R8','R9'),(1.43333,.98212)):
        selected=sorted((r for r in rows if r[0]==name),key=lambda r:r[2])
        ranged=next(r for r in selected if r[1]=='Optical + range')
        ax.scatter(ranged[2],ranged[6],marker='x',c='black',label='Optical + range')
        selected=[r for r in selected if r is not ranged]
        ax.plot([r[2] for r in selected],[r[6] for r in selected],'o-',label='Simulated')
        ax.axhline(observed,c='tab:orange',ls='--',label='Measured')
        ax.set(xlabel='Cube yaw correction (°)',ylabel='Peak packet magnitude (g)',title=name)
        ax.legend()
    fig.savefig(here/'yaw-transfer.png',dpi=150)

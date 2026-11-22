"""Test the robot model's torque-limited lift on a real roll and a no-disturbance approach."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from tread_speed import run,load_rows
from impact_loads import observe,trace

here=Path(__file__).resolve().parent
rest=np.genfromtxt(here.parents[1]/'unit3-cube-accelerometer/resting.csv',names=True,delimiter=',')
gain=np.linalg.norm([rest[k+'_counts'].mean() for k in 'xyz'])/8192
fixture=sys.argv[1]
fig,axes=plt.subplots(2,1,figsize=(9,6),layout='constrained')
with (here/'servo-load.csv').open('w') as f:
    writer=csv.writer(f,lineterminator='\n')
    writer.writerow(('run','articulated_lift','dt_ms','peak_fork_N','peak_shelf_N','peak_packet_g','counter_changes',
                     'peak_cube_tilt_deg','last_cube_tilt_deg','max_height_tracking_error_mm'))
    for idx,directory in enumerate([here,here.parent/'R9']):
        rotation=None
        if idx:
            cube=next(r for r in load_rows('roll-poses.csv',directory) if r['source']=='robot_observed_object' and float(r['sec'])<0)
            raw=Rotation.from_quat([float(cube[k]) for k in ('qw','qx','qy','qz')],scalar_first=True)
            vertical=raw.inv().apply([0,0,1])
            axis=np.eye(3)[np.argmax(abs(vertical))]*np.sign(vertical[np.argmax(abs(vertical))])
            correction,_=Rotation.align_vectors([[0,0,1]],[raw.apply(axis)])
            rotation=correction*raw
        for articulated,dt in ([(False,.00025),(True,.0005),(True,.00025)] if not idx else [(True,.00025)]):
            trace.clear()
            height=[]
            def record(m,d):
                observe(m,d)
                if articulated:
                    target=d.actuator('lift').ctrl[0]
                    actual=d.joint('lift_joint').qpos[0]
                    height.append(66*(np.sin(actual)-np.sin(target)))
            t,tilt,_,_,sensor=run(fixture,dt,True,directory,rotation,{'cube_chip_yaw_deg':180,'cube_accel_scale':[gain]*3},
                                  observer=record,articulated=articulated)
            a=np.asarray(trace)
            ts=np.repeat([stamp for stamp,_ in sensor.messages],3)
            raw=np.vstack([np.frombuffer(b[2:],'<i2').reshape(3,3) for _,b in sensor.messages])/8192
            norm=np.linalg.norm(raw,axis=1)
            writer.writerow((directory.name,articulated,dt*1000,a[:,3].max(),a[:,1].max(),norm[ts>2.5].max(),
                             sum(stamp>2.5 for stamp in sensor.taps.events),max(tilt),tilt[-1],max(map(abs,height)) if height else 0))
            if dt==.00025:
                axes[idx].plot(ts,norm,'.',ms=2,label='Servo lift' if articulated else 'Prescribed fork')
        records=np.genfromtxt(directory/'cube.csv.gz',names=True,delimiter=',')
        zero=next(float(r['sec']) for r in load_rows('actions.csv',directory) if r['action']=='roll cube')
        phys=np.linalg.norm(np.column_stack([records[k] for k in 'xyz']),axis=1)/8192
        axes[idx].plot(records['sec']-zero,phys,'.',ms=2,label='Physical cube')
        axes[idx].set(title=directory.name+' · '+('native roll' if not idx else 'visually failed approach'),
                      xlim=(2.5,5.21 if not idx else 19),ylabel='Packet magnitude (g)')
        axes[idx].legend()
axes[-1].set_xlabel('Seconds after roll command')
fig.savefig(here/'servo-load.png',dpi=150)

"""Test frozen R8 fork models on the later R11 native roll miss."""
import csv
import gzip
from pathlib import Path
import sys
import tempfile

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'R8'))
from tread_speed import run,load_rows


def prepare(folder, start, end):
    for filename in ('actions.csv','state.csv.gz'):
        (folder/filename).symlink_to(HERE/filename)
    state=np.genfromtxt(HERE/'state.csv.gz',delimiter=',',names=True)
    with gzip.open(HERE/'poses.csv.gz','rt') as f:
        poses=list(csv.DictReader(f))
    cube=next(r for r in reversed(poses) if r['source']=='robot_observed_object' and float(r['sec'])<start)
    def rotation(row):
        return Rotation.from_quat([float(row[k]) for k in ('qw','qx','qy','qz')],scalar_first=True)
    initial=rotation(cube)
    vertical=initial.inv().apply([0,0,1])
    axis=np.eye(3)[np.argmax(abs(vertical))]*np.sign(vertical[np.argmax(abs(vertical))])
    level,_=Rotation.align_vectors([[0,0,1]],[initial.apply(axis)])
    with (folder/'roll-poses.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['sec','source','x_mm','y_mm','yaw_rad','pitch_rad','lift_mm'])
        for row in poses:
            t=float(row['sec'])
            if row!=cube and (row['source']!='robot_state' or not start-.3<=t<=end+1):continue
            assert row['origin_id']==cube['origin_id']
            j=np.argmin(abs(state['sec']-t))
            if row!=cube: assert abs(state['sec'][j]-t)<1e-5
            yaw=rotation(row).as_euler('ZYX')[0]
            w.writerow([t-start,row['source'],row['x_mm'],row['y_mm'],yaw,state['pitch'][j],state['lift'][j]])
    return level*initial


def main(fixture):
    rows=load_rows('actions.csv',HERE)
    start,end=[float(r['sec']) for r in rows if r['action']=='roll cube']
    physical=np.genfromtxt(HERE/'cube.csv.gz',names=True,delimiter=',')
    pt=physical['sec']-start
    norm=np.linalg.norm(np.column_stack([physical[k] for k in 'xyz']),axis=1)/8192
    rest=np.genfromtxt(HERE.parents[1]/'unit3-cube-accelerometer/resting.csv',names=True,delimiter=',')
    gain=np.linalg.norm([rest[k+'_counts'].mean() for k in 'xyz'])/8192
    fig,ax=plt.subplots(figsize=(9,4),layout='constrained')
    keep=(pt>=0)&(pt<=end-start)
    ax.scatter(pt[keep],norm[keep],s=10,color='black',label='Physical cube')
    with tempfile.TemporaryDirectory() as tmp,(HERE/'miss-replay.csv').open('w') as out:
        folder=Path(tmp);initial=prepare(folder,start,end)
        w=csv.writer(out,lineterminator='\n')
        w.writerow(['fork','dt_ms','fork_contact_steps','tread_contact_steps','peak_tilt_deg','peak_packet_g','tap_changes'])
        for articulated in (False,True):
            for dt in (.0005,.00025):
                fork_steps=[]
                def observe(m,d):
                    if d.time<=end-start:
                        pair={m.body('cube').id,m.body('fork').id}
                        fork_steps.append(any(set(m.geom_bodyid[c.geom])==pair for c in d.contact))
                t,tilt,hit,_,chip=run(fixture,dt,True,folder,initial,{'cube_chip_yaw_deg':180,'cube_accel_scale':[gain]*3},
                                      observer=observe,articulated=articulated)
                pack=[(s,b) for s,b in chip.messages if s<=end-start]
                ts=np.repeat([s for s,_ in pack],3)
                g=np.linalg.norm(np.vstack([np.frombuffer(b[2:],'<i2').reshape(3,3) for _,b in pack]).astype(float),axis=1)/8192
                label='Torque-limited lift' if articulated else 'Prescribed fork'
                keep=t+dt<=end-start
                result=(label,dt*1000,sum(fork_steps),hit[keep].sum(),tilt[keep].max(),g.max(),sum(s<=end-start for s in chip.taps.events))
                print(result);w.writerow(result)
                if dt==.00025:ax.plot(ts,g,'.',ms=2,label=label)
    ax.set(xlabel='Seconds after native roll command',ylabel='Packet specific force (g)')
    ax.legend();fig.savefig(HERE/'miss-replay.png',dpi=150)


if __name__=='__main__':
    main(sys.argv[1])

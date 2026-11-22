"""Vary rubber response time only after the last fork contact of the R8 replay."""
from pathlib import Path
import csv
import sys

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from tread_speed import run, load_rows

HERE = Path(__file__).resolve().parent


def trial(fixture, response, overrides):
    released = None
    records = []
    def observe(m,d):
        nonlocal released
        if d.time < 4.765:
            return
        if released is None:
            released = d.joint('cube_root').qvel.copy()
            rubber = [i for i in range(m.ngeom) if (m.geom(i).name or '').startswith('cube_rubber_')]
            assert len(rubber)==8 and np.allclose(m.geom_solref[rubber,0],.008)
            m.geom_solref[rubber,0] = response*.001
        cube = m.body('cube').id
        load = np.zeros(3)
        depth = 0.
        for j,c in enumerate(d.contact):
            bodies = m.geom_bodyid[c.geom]
            if cube not in bodies:
                continue
            assert 0 in bodies, 'Another robot contact confounds the landing'
            wrench = np.zeros(6)
            mj.mj_contactForce(m,d,j,wrench)
            load += (1 if bodies[1]==cube else -1)*(wrench[:3]@c.frame.reshape(3,3))
            depth = max(depth,-c.dist)
        records.append((d.time, np.linalg.norm(load), depth*1000, np.linalg.norm(d.sensor('chip').data)/9.81))
    _,tilt,_,_,chip = run(fixture,.00025,True,sensor_overrides=overrides,observer=observe)
    values = [(t,np.linalg.norm(np.frombuffer(payload[2:],'<i2').reshape(3,3).astype(float),axis=1).max()/8192)
              for t,payload in chip.messages if 4.765<=t<5.]
    return released,np.array(records),np.array(values),tilt[-1]


def main(fixture):
    rest = np.genfromtxt(HERE.parents[1]/'unit3-cube-accelerometer/resting.csv',names=True,delimiter=',')
    gain = np.linalg.norm([rest[k+'_counts'].mean() for k in 'xyz'])/8192
    physical = np.genfromtxt(HERE/'cube.csv.gz',names=True,delimiter=',')
    zero = next(float(r['sec']) for r in load_rows('actions.csv') if r['action']=='roll cube')
    times = physical['sec']-zero
    norm = np.linalg.norm(np.column_stack([physical[k] for k in 'xyz']),axis=1)/8192
    common = (times>=4.765)&(times<5)
    print('Physical landing-window max',norm[common].max())
    fig,axes = plt.subplots(2,1,figsize=(9,6),sharex=True,layout='constrained')
    rows = []
    initial = None
    for response in (4,8,16,32,64):
        qvel,a,pack,angle = trial(fixture,response,{'cube_chip_yaw_deg':180,'cube_accel_scale':[gain]*3})
        if initial is None:
            initial = qvel
        np.testing.assert_array_equal(initial,qvel)
        peak = pack[:,1].max()
        if response==8:
            assert abs(peak-5.32117476)<.005
        row = (response,qvel[2],np.linalg.norm(qvel[3:]),a[:,1].max(),a[:,2].max(),a[:,3].max(),peak,angle)
        rows.append(row)
        print(np.round(row,3))
        axes[0].plot(a[:,0],a[:,1],label=f'{response} ms')
        axes[1].plot(pack[:,0],pack[:,1],'.-')
    axes[1].scatter(times[common],norm[common],s=16,color='black',label='Physical packet slots')
    axes[0].set(ylabel='Shelf force (N)',title='R8 landing from the same release state')
    axes[0].legend(ncol=5)
    axes[1].set(xlabel='Time after roll command (s)',ylabel='Packet acceleration (g)',xlim=(4.765,5))
    axes[1].legend()
    fig.savefig(HERE/'landing-compliance.png',dpi=160)
    with (HERE/'landing-compliance.csv').open('w') as out:
        w=csv.writer(out,lineterminator='\n')
        w.writerow(['rubber_ms','release_vz_m_s','release_angular_speed_rad_s','peak_shelf_N','max_penetration_mm','peak_raw_g','peak_packet_g','final_tilt_deg'])
        w.writerows(rows)


if __name__=='__main__':
    main(sys.argv[1])

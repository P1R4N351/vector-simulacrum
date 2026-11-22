"""Check the centered-chip assumption on the fixed R8 roll, without fitting a location."""
from itertools import product
from pathlib import Path
import csv
import sys

import numpy as np
import matplotlib.pyplot as plt
from tread_speed import run, load_rows

HERE=Path(__file__).resolve().parent


def main(fixture):
    rest=np.genfromtxt(HERE.parents[1]/'unit3-cube-accelerometer/resting.csv',delimiter=',',names=True)
    gain=np.linalg.norm([rest[k+'_counts'].mean() for k in 'xyz'])/8192
    measured=np.genfromtxt(HERE/'cube.csv.gz',names=True,delimiter=',')
    zero=next(float(r['sec']) for r in load_rows('actions.csv') if r['action']=='roll cube')
    t=measured['sec']-zero
    hardware=np.linalg.norm(np.column_stack([measured[k] for k in 'xyz']),axis=1)/8192
    fig,ax=plt.subplots(figsize=(9,4.5),layout='constrained')
    rows=[]
    first=None
    for xyz in [(0,0,0),*product((-15,15),repeat=3)]:
        _,tilt,_,_,chip=run(fixture,.00025,True,sensor_overrides={'cube_chip_yaw_deg':180,
                    'cube_chip_position_mm':xyz,'cube_accel_scale':[gain]*3})
        if first is None:first=tilt
        np.testing.assert_array_equal(tilt,first)
        times=np.repeat([stamp for stamp,b in chip.messages],3)
        g=np.linalg.norm(np.vstack([np.frombuffer(b[2:],'<i2').reshape(3,3) for stamp,b in chip.messages]).astype(float),axis=1)/8192
        pre=g[(times>=4.60)&(times<4.78125)]
        peak=g[(times>=4.5)&(times<5)].max()
        rows.append((*xyz,pre.min(),np.sum(pre<.5),peak))
        keep=(times>=4.60)&(times<4.92)
        ax.scatter(times[keep],g[keep],s=8,alpha=.6 if xyz==(0,0,0) else .16,
                   color='tab:blue' if xyz==(0,0,0) else 'tab:orange',
                   label='Centered chip' if xyz==(0,0,0) else ('Offset chips' if len(rows)==2 else None))
    keep=(t>=4.60)&(t<4.92)
    ax.scatter(t[keep],hardware[keep],s=20,marker='x',color='black',label='Physical cube')
    ax.set(xlabel='Seconds from roll command',ylabel='Packet specific force (g)')
    ax.legend()
    fig.savefig(HERE/'chip-offset.png',dpi=150)
    with (HERE/'chip-offset.csv').open('w') as out:
        w=csv.writer(out,lineterminator='\n')
        w.writerow(['x_mm','y_mm','z_mm','pre_min_g','pre_slots_below_half_g','landing_peak_g'])
        w.writerows(rows)
    print(np.round(rows,3))


if __name__=='__main__':
    main(sys.argv[1])

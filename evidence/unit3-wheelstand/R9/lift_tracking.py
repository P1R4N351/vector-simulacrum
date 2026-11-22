"""Compare loaded lift encoder tracking with and without damping compensation."""
import csv
from concurrent.futures import ProcessPoolExecutor
import sys

import matplotlib.pyplot as plt
import numpy as np
from yaw_transfer import cube_rotation, here, load_rows, run


def replay(case):
    name, feedforward, path = case
    directory = here.parent / name
    view = 'Before roll' if name == 'R8' else 'Next approach'
    yaw = next(float(r['grid_yaw_minus_sdk_deg']) for r in load_rows('optical-registration.csv', here)
               if r['calibration'] == 'Dock and first cube' and r['view'] == view)
    height = []
    def observe(model, data):
        height.append(45 + 66*np.sin(data.joint('lift_joint').qpos[0]))
    t, tilt, _, _, sensor = run(path, .00025, True, directory, cube_rotation(directory, yaw),
                                {'cube_chip_yaw_deg':180,'cube_accel_scale':[.970709]*3},
                                observer=observe, articulated=True, shelf_support=True,
                                lift_velocity_feedforward=feedforward)
    states = sorted((float(r['sec']),float(r['lift_mm'])) for r in load_rows('roll-poses.csv', directory)
                    if r['source'] == 'robot_state')
    stamps, enc = np.array(states).T
    keep = (stamps>=4) & (stamps<=min(5.208,t[-1]))
    error = np.interp(stamps[keep],t,height) - enc[keep]
    packets = np.vstack([np.frombuffer(b[2:],'<i2').reshape(3,3) for ts,b in sensor.messages if ts>2.5])/8192
    return (name, feedforward, np.sqrt(np.mean(error**2)), abs(error).max(), tilt.max(),
            np.linalg.norm(packets,axis=1).max(), sum(ts>2.5 for ts in sensor.taps.events)), (t,height,stamps,enc)


if __name__=='__main__':
    fig, axes = plt.subplots(2,1,figsize=(8,6),layout='constrained')
    with ProcessPoolExecutor(2) as pool, (here/'lift-tracking.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(('run','velocity_feedforward','height_rmse_mm','height_max_error_mm','peak_tilt_deg','peak_packet_g','taps'))
        for scores,(t,h,stamps,enc) in pool.map(replay, [(n,b,sys.argv[1]) for n in ('R8','R9') for b in (False,True)]):
            w.writerow(scores)
            name, enabled=scores[:2]
            ax=axes[int(name=='R9')]
            ax.plot(t,h,label='Velocity compensation' if enabled else 'Position servo')
            if enabled:
                ax.plot(stamps,enc,'k.',label='Physical encoder')
            ax.set(xlim=(3.8,5.21),title=name,ylabel='Lift height (mm)')
            ax.set_ylim(25,82)
            ax.legend()
    axes[-1].set_xlabel('Seconds from roll command')
    fig.savefig(here/'lift-tracking.png',dpi=150)

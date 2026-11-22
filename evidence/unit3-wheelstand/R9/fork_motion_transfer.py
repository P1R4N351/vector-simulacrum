"""Isolate enforcing encoder-derived fork motion on the revised free cube."""
import csv
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import matplotlib.pyplot as plt
import numpy as np
from surface_transfer import replace_boxes
from yaw_transfer import cube_rotation, here, load_rows, run


def clip_height(directory, dest):
    dest.mkdir()
    for name in ('state.csv.gz','actions.csv'):
        (dest/name).symlink_to(directory/name)
    states = load_rows('roll-poses.csv',directory)
    with (dest/'roll-poses.csv').open('w') as file:
        writer = csv.DictWriter(file,fieldnames=states[0],lineterminator='\n')
        writer.writeheader()
        for row in states:
            if row['source']=='robot_state':
                row['lift_mm'] = max(32.,min(92.,float(row['lift_mm'])))
            writer.writerow(row)


def compare(fixture):
    fig, axes = plt.subplots(2, 2, figsize=(9, 5.5), layout='constrained')
    with (here/'fork-motion-transfer.csv').open('w') as file:
        writer = csv.writer(file, lineterminator='\n')
        writer.writerow(['run','drive','peak_deg','final_deg','peak_g','tap_changes'])
        for col, (name, view) in enumerate([('R8','Before roll'),('R9','Next approach')]):
            directory = here.parent / name
            yaw = next(float(r['grid_yaw_minus_sdk_deg']) for r in load_rows('optical-registration.csv',here)
                       if r['calibration']=='Dock and first cube' and r['view']==view)
            clipped = fixture.parent/name
            clip_height(directory,clipped)
            for articulated, label, inputs in [(True,'Torque-limited',directory),
                                                (False,'Encoder-prescribed',directory),
                                                (False,'Prescribed within stops',clipped)]:
                t, tilt, _, _, sensor = run(fixture,.00025,True,inputs,cube_rotation(directory,yaw),
                    {'cube_chip_yaw_deg':180,'cube_accel_scale':[.970709]*3},
                    articulated=articulated,shelf_support=True)
                packets = [(s,np.frombuffer(b[2:],'<i2').reshape(3,3)/8192)
                           for s,b in sensor.messages if s>2.5]
                stamps, samples = zip(*packets)
                peak = np.linalg.norm(samples,axis=2).max()
                writer.writerow([name,label,tilt.max(),tilt[-1],peak,sum(s>2.5 for s in sensor.taps.events)])
                axes[0,col].plot(t,tilt,label=label)
                axes[1,col].plot(stamps,np.mean(samples,axis=1)[:,1],label=label)
            zero = next(float(r['sec']) for r in load_rows('actions.csv',directory) if r['action']=='roll cube')
            raw = np.genfromtxt(directory/'cube.csv.gz',names=True,delimiter=',')
            axes[1,col].plot(raw['sec'][::3]-zero,raw['y'].reshape(-1,3).mean(axis=1)/8192,
                             'k.',ms=2,label='Physical')
            axes[0,col].set_title(name)
            axes[1,col].set_xlabel('Seconds from command')
            for ax in axes[:,col]:
                ax.set_xlim(2.5,t[-1])
    axes[0,0].set_ylabel('Simulated cube tilt (°)')
    axes[1,0].set_ylabel('Cube packet Y (g)')
    axes[1,1].legend()
    fig.savefig(here/'fork-motion-transfer.png',dpi=160)


if __name__ == '__main__':
    with TemporaryDirectory() as directory:
        fixture = Path(directory)/'revised.xml'
        replace_boxes(sys.argv[1],fixture)
        compare(fixture)

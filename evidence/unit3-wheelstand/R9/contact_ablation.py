"""Isolate fork and tread contacts in the encoder-prescribed roll diagnostic."""
import csv
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

import matplotlib.pyplot as plt
import numpy as np

from surface_transfer import replace_boxes
from yaw_transfer import cube_rotation, here, load_rows, run


def trial(fixture, directory, rotation, disabled):
    configured = False
    def observe(model, data):
        nonlocal configured
        if configured:
            return
        for name in disabled:
            select = model.geom_bodyid == model.body(name).id
            model.geom_contype[select] = 0
            model.geom_conaffinity[select] = 0
        configured = True
    return run(fixture,.00025,True,directory,rotation,
               {'cube_chip_yaw_deg':180,'cube_accel_scale':[.970709]*3},
               observer=observe,shelf_support=True)


def compare(fixture):
    fig, axes = plt.subplots(2,2,figsize=(9,5.5),layout='constrained')
    with (here/'contact-ablation.csv').open('w') as file:
        writer = csv.writer(file,lineterminator='\n')
        writer.writerow(['recording','enabled','max_tilt_deg','final_tilt_deg','peak_g','tap_changes'])
        for col,(name,view) in enumerate([('R8','Before roll'),('R9','Next approach')]):
            directory = here.parent/name
            yaw = next(float(r['grid_yaw_minus_sdk_deg']) for r in load_rows('optical-registration.csv',here)
                       if r['calibration']=='Dock and first cube' and r['view']==view)
            rotation = cube_rotation(directory,yaw)
            for label,disabled in [('Both',()),('Fork only',('tread',)),('Tread only',('fork',)),('Neither',('fork','tread'))]:
                t,tilt,_,_,sensor = trial(fixture,directory,rotation,disabled)
                packets = [(s,np.frombuffer(b[2:],'<i2').reshape(3,3)/8192) for s,b in sensor.messages if s>2.5]
                stamps,samples = zip(*packets)
                peak = np.linalg.norm(samples,axis=2).max()
                writer.writerow([name,label,tilt.max(),tilt[-1],peak,sum(s>2.5 for s in sensor.taps.events)])
                axes[0,col].plot(t,tilt,label=label)
                axes[1,col].plot(stamps,np.mean(samples,axis=1)[:,1],label=label)
            raw = np.genfromtxt(directory/'cube.csv.gz',names=True,delimiter=',')
            zero = next(float(r['sec']) for r in load_rows('actions.csv',directory) if r['action']=='roll cube')
            axes[1,col].plot(raw['sec'][::3]-zero,raw['y'].reshape(-1,3).mean(axis=1)/8192,'k.',ms=2,label='Recorded')
            for ax in axes[:,col]:
                ax.set_xlim(2.5,t[-1])
            axes[0,col].set_title(name)
            axes[1,col].set_xlabel('Seconds after command')
    axes[0,0].set_ylabel('Simulated tilt (°)')
    axes[1,0].set_ylabel('Cube packet Y (g)')
    axes[1,1].legend(fontsize=8)
    fig.savefig(here/'contact-ablation.png',dpi=160)


if __name__ == '__main__':
    with TemporaryDirectory() as tmp:
        fixture = Path(tmp)/'revised.xml'
        replace_boxes(sys.argv[1],fixture)
        compare(fixture)

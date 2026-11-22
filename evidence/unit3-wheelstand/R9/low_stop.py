"""Transfer the independent unloaded lift minimum into two roll replay hypotheses."""
import csv
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import matplotlib.pyplot as plt
import numpy as np
from yaw_transfer import cube_rotation, here, load_rows, run


def angle(h):
    return np.arcsin((h-45)/66)


def height(a):
    return 45+66*np.sin(a)


def compare(name, minimum, ax):
    directory=here.parent/name
    states=load_rows('roll-poses.csv',directory)
    view='Before roll' if name=='R8' else 'Next approach'
    yaw=next(float(r['grid_yaw_minus_sdk_deg']) for r in load_rows('optical-registration.csv',here)
             if r['calibration']=='Dock and first cube' and r['view']==view)
    rotation=cube_rotation(directory,yaw)
    obs=sorted((float(r['sec']),float(r['lift_mm'])) for r in states if r['source']=='robot_state')
    stamps,measured=np.array(obs).T
    rows=[]
    for mode in ('Nominal','Travel','Encoder zero'):
        bias=angle(32)-angle(minimum) if mode=='Encoder zero' else 0.
        actual=[]
        def observe(model,data):
            actual.append(height(data.joint('lift_joint').qpos[0]-bias))
        with TemporaryDirectory() as d:
            trial=Path(d)
            for f in ('actions.csv','state.csv.gz'):
                (trial/f).symlink_to(directory/f)
            with (trial/'roll-poses.csv').open('w') as f:
                w=csv.DictWriter(f,fieldnames=states[0],lineterminator='\n')
                w.writeheader()
                for r in states:
                    if r['source']=='robot_state' and bias:
                        r=dict(r,lift_mm=height(angle(float(r['lift_mm']))+bias))
                    w.writerow(r)
            overrides={'cube_chip_yaw_deg':180,'cube_accel_scale':[.970709]*3}
            if mode=='Travel':
                overrides['lift_height_min_mm']=minimum
            t,tilt,_,_,sensor=run(sys.argv[1],.00025,True,trial,rotation,overrides,observer=observe,
                                  articulated=True,shelf_support=True)
        keep=(stamps>=4)&(stamps<=min(5.208,t[-1]))
        err=np.interp(stamps[keep],t,actual)-measured[keep]
        packets=np.vstack([np.frombuffer(b[2:],'<i2').reshape(3,3) for s,b in sensor.messages if s>2.5])/8192
        rows.append((name,mode,np.sqrt(np.mean(err**2)),tilt.max(),np.linalg.norm(packets,axis=1).max(),
                     sum(s>2.5 for s in sensor.taps.events)))
        ax.plot(t,actual,label=mode)
    ax.plot(stamps,measured,'k.',label='Encoder')
    ax.set(xlim=(4,5.21),ylim=(25,75) if name=='R8' else (27,35),title=name,ylabel='Reported lift height (mm)')
    ax.legend(fontsize=8)
    return rows


if __name__=='__main__':
    samples=[json.loads(s) for s in (here.parents[1]/'docked-actuator-timing/unit3.jsonl').read_text().splitlines()]
    minimum=min(s['lift'] for s in samples if s['k']=='s')
    fig,axes=plt.subplots(2,1,figsize=(8,6),layout='constrained')
    with (here/'low-stop.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(('run','hypothesis','height_rmse_mm','peak_tilt_deg','peak_packet_g','taps'))
        for name,ax in zip(('R8','R9'),axes):
            w.writerows(compare(name,minimum,ax))
    axes[-1].set_xlabel('Time after roll command (s)')
    fig.savefig(here/'low-stop.png',dpi=150)

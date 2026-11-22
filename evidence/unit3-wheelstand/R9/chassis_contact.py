"""Paired test of the omitted chassis collision surface at fixed replay settings."""
from concurrent.futures import ProcessPoolExecutor
import csv
import sys

import matplotlib.pyplot as plt
import numpy as np
from yaw_transfer import cube_rotation, here, load_rows, run


def replay(case):
    name,enabled,path,*extra=case
    pieces=extra[0] if extra else None
    directory=here.parent/name
    view='Before roll' if name=='R8' else 'Next approach'
    yaw=next(float(r['grid_yaw_minus_sdk_deg']) for r in load_rows('optical-registration.csv',here)
             if r['calibration']=='Dock and first cube' and r['view']==view)
    contacts=[]
    surface_ids=set()
    def observe(model,data):
        if enabled:
            if not surface_ids:
                names=['chassis_geom'] if pieces is None else [f'apron_{i}' for i in range(len(pieces))]
                surface_ids.update(model.geom(key).id for key in names)
            cid=model.body('cube').id
            if any(surface_ids.intersection(c.geom) and cid in model.geom_bodyid[c.geom] for c in data.contact):
                contacts.append(data.time)
    t,tilt,_,_,sensor=run(path,.00025,True,directory,cube_rotation(directory,yaw),
        {'cube_chip_yaw_deg':180,'cube_accel_scale':[.970709]*3},observer=observe,
        articulated=True,shelf_support=True,chassis_contact=enabled is True,apron_pieces=pieces)
    samples=[np.frombuffer(b[2:],'<i2').reshape(3,3)/8192 for s,b in sensor.messages if s>2.5]
    stamps=[s for s,b in sensor.messages if s>2.5]
    counts=np.array(samples)
    return (name,enabled,min(contacts,default=None),len(contacts)*.00025,max(tilt),tilt[-1],
            np.linalg.norm(counts,axis=2).max(),sum(s>2.5 for s in sensor.taps.events)), (t,tilt,stamps,counts.mean(axis=1))


if __name__=='__main__':
    fig,axes=plt.subplots(2,1,figsize=(8,6),layout='constrained',sharex=True)
    with ProcessPoolExecutor(2) as pool,(here/'chassis-contact.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(('run','chassis_contact','first_chassis_contact_s','total_chassis_contact_s','peak_cube_tilt_deg','last_tilt_deg','peak_packet_g','tap_changes'))
        for scores,(t,tilt,stamps,acc) in pool.map(replay,[(n,e,sys.argv[1]) for n in ('R8','R9') for e in (False,True)]):
            w.writerow(scores)
            if scores[0]!='R8':continue
            label='With chassis box' if scores[1] else 'Fork and tread'
            axes[0].plot(t,tilt,label=label)
            axes[1].plot(stamps,acc[:,1],label=label)
    raw=here.parent/'R8'
    c=np.genfromtxt(raw/'cube.csv.gz',delimiter=',',names=True)
    zero=next(float(r['sec']) for r in load_rows('actions.csv',raw) if r['action']=='roll cube')
    axes[1].plot(c['sec'][::3]-zero,c['y'].reshape(-1,3).mean(axis=1)/8192,'k.',ms=2,label='Physical')
    axes[0].set(ylabel='Simulated cube tilt (°)')
    axes[1].set(xlabel='Seconds after roll command',ylabel='Cube packet Y (g)',xlim=(2.8,5.3))
    for ax in axes:ax.legend()
    fig.savefig(here/'chassis-contact.png',dpi=150)

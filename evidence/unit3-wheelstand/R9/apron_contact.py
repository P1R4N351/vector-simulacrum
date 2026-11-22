"""Drive the registered apron against the free cube using the chassis material."""
from concurrent.futures import ProcessPoolExecutor
import csv
import json
import subprocess
import sys

import matplotlib.pyplot as plt
import numpy as np

from chassis_contact import replay
from yaw_transfer import here, load_rows


def apron_pieces():
    import coacd
    source=json.loads(subprocess.check_output(['node',here/'shell_triangles.mjs']))
    verts,indices=np.unique(np.round(source['front apron'],9),axis=0,return_inverse=True)
    faces=indices.reshape(-1,3)
    tri=verts[faces]
    valid=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)>1e-14
    hulls=coacd.run_coacd(coacd.Mesh(verts*1000,faces[valid]),threshold=.025,seed=0)
    return [points/1000 for points,_ in hulls]


if __name__=='__main__':
    pieces=apron_pieces()
    print('Convex pieces:',len(pieces))
    cases=[(n,mode,sys.argv[1],pieces if mode=='apron' else None)
           for n in ('R8','R9') for mode in (False,True,'apron')]
    fig,axes=plt.subplots(2,1,figsize=(8,6),sharex=True,layout='constrained')
    with ProcessPoolExecutor(2) as pool,(here/'apron-contact.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(('run','contact_surface','onset_s','duration_s','peak_tilt_deg','last_tilt_deg','peak_packet_g','taps'))
        for scores,(t,tilt,stamps,acc) in pool.map(replay,cases):
            label={True:'Chassis box',False:'Fork and tread','apron':'Curved apron'}[scores[1]]
            w.writerow((scores[0],label,*scores[2:]))
            if scores[0]=='R8':
                axes[0].plot(t,tilt,label=label)
                axes[1].plot(stamps,acc[:,1],label=label)
    raw=here.parent/'R8'
    c=np.genfromtxt(raw/'cube.csv.gz',delimiter=',',names=True)
    zero=next(float(r['sec']) for r in load_rows('actions.csv',raw) if r['action']=='roll cube')
    axes[1].plot(c['sec'][::3]-zero,c['y'].reshape(-1,3).mean(axis=1)/8192,'k.',ms=2,label='Physical')
    axes[0].set_ylabel('Simulated cube tilt (°)')
    axes[1].set(xlabel='Seconds after roll command',ylabel='Cube Y (g)',xlim=(2.8,5.3))
    for ax in axes:ax.legend()
    fig.savefig(here/'apron-contact.png',dpi=150)

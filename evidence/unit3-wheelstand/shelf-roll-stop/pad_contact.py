"""Audit the simulated pad and shell contacts in one representative held-out release."""
import csv
import mujoco
import numpy as np
from release import here, initial_state

with (here/'pad-contact.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['pad_ms','material','first_contact_ms','max_overlap_mm','summed_normal_impulse_Ns'])
    for ms in (8,32):
        m,d,_,_=initial_state(0,15.76)
        m.opt.timestep=.000125
        pads=[i for i in range(m.ngeom) if m.geom(i).name.startswith('cube_rubber_')]
        m.geom_solref[pads,0]=ms*.001
        d.joint('cube_root').qpos[2]+=.008
        stats={}
        force=np.empty(6)
        for _ in range(round(.15/m.opt.timestep)):
            mujoco.mj_step(m,d)
            for i,c in enumerate(d.contact):
                names=[m.geom(g).name for g in c.geom]
                name=next((n for n in names if n.startswith('cube_')),None)
                if not name or 'table' not in names: continue
                mujoco.mj_contactForce(m,d,i,force)
                if force[0]<=0: continue
                stat=stats.setdefault(name.split('_')[1],[d.time*1000,0.,0.])
                stat[1]=max(stat[1],-1000*c.dist)
                stat[2]+=force[0]*m.opt.timestep
        for k,stat in stats.items(): w.writerow([ms,k,*stat])

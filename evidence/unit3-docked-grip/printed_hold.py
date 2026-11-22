"""Replay the measured wrist trajectory with several face-square free-cube placements."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from simulacrum import params

root = Path(__file__).resolve().parent
sys.path.insert(0,str(root.parent/"unit3-cube-accelerometer"))
from pickup_support import contact_load

record = np.genfromtxt(root/'encoder.csv', names=True, delimiter=',')
v = params.values(params.load(model='vector_2'))
bx,_,bz = v['lift_base_position_mm']
t = np.arange(-.35,1.8,.001)
h = np.interp(t,record['seconds'],record['lift_mm'])
wrist = .001*np.column_stack([bx+np.sqrt(v['lift_arm_length_mm']**2-(h-bz)**2),np.zeros_like(h),h])
fig, axes = plt.subplots(1,2,figsize=(10,4),layout='constrained',sharey=True)
rows = []
for ax,(name,path) in zip(axes,[('Three boxes',root.parent/'unit3-cube-contact-replay/fixture.xml'),('Printed STL',sys.argv[1])]):
    model=mj.MjModel.from_xml_path(str(path))
    fork,cube = [model.body(n).id for n in ('fork','cube')]
    assert abs(model.opt.timestep-.001)<1e-9
    ids=np.flatnonzero((model.geom_bodyid==fork)&(model.geom_contype!=0))
    sizes=[]
    init=mj.MjData(model)
    init.mocap_pos[0]=wrist[0]
    mj.mj_forward(model,init)
    for g in ids:
        rotation=init.geom_xmat[g].reshape(3,3)[0]
        if model.geom_type[g] == mj.mjtGeom.mjGEOM_BOX:
            extent=abs(rotation)@model.geom_size[g]
        else:
            m=model.geom_dataid[g]
            vertices=model.mesh_vert[model.mesh_vertadr[m]:model.mesh_vertadr[m]+model.mesh_vertnum[m]]
            extent=np.max(vertices@rotation)
        sizes.append(init.geom_xpos[g,0]+extent)
    for lateral in (-3,0,3):
        for gap in (-3,-1,0,1):
            d=mj.MjData(model)
            d.mocap_pos[0]=wrist[0]
            d.joint('cube_root').qpos[:]=[max(sizes)+.02215+gap*.001,lateral*.001,.02215,1,0,0,0]
            mj.mj_forward(model,d)
            def cross(c):
                return set(model.geom_bodyid[c.geom])=={fork,cube}
            penetration=max([0.,*[-c.dist*1000 for c in d.contact if cross(c)]])
            if penetration>1e-5:
                rows.append([name,lateral,gap,penetration,None,None,None,None,None])
                continue
            angles,heights,touch,load=[],[],[],[]
            for clock,pos in zip(t,wrist):
                d.mocap_pos[0]=pos
                mj.mj_step(model,d)
                angles.append(np.degrees(np.arccos(np.clip(d.xmat[cube,8],-1,1))))
                heights.append(d.xpos[cube,2]*1000)
                if 1.1 <= clock < 1.35:
                    forces=contact_load(model,d,cube,model.geom("table").id,fork)
                    load.append(sum(forces[1:3])/(model.body_mass[cube]*np.linalg.norm(model.opt.gravity)))
                touch.append(any(c.efc_address>=0 and cross(c) for c in d.contact))
            angles=np.array(angles)
            hold=(t>=1.1)&(t<1.35)
            rows.append([name,lateral,gap,penetration,angles[t>=0].max(),angles[hold].max(),
                         np.mean(np.array(touch)[hold]),max(np.array(heights)[hold])-heights[np.searchsorted(t,0)],np.mean(load)])
            ax.plot(t,angles,lw=1,alpha=.7)
    ax.axvspan(1.1,1.35,color='orange',alpha=.2,label='Physical raised interval')
    ax.set(title=name,xlabel='Seconds from lift command',xlim=(0,1.65))
    ax.legend(fontsize=8)
axes[0].set_ylabel('Simulated cube inclination (degrees)')
fig.savefig(root/'printed-hold.png',dpi=165)
with (root/'printed-hold.csv').open('w') as f:
    w=csv.writer(f)
    w.writerow('fork lateral_mm envelope_gap_mm initial_penetration_mm peak_deg raised_peak_deg raised_contact_fraction raised_max_rise_mm mean_upward_fork_force_weights'.split())
    w.writerows(rows)

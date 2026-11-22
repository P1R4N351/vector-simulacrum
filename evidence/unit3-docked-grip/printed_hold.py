"""Replay the measured wrist trajectory with several face-square free-cube placements."""
import argparse
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

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('fixture')
parser.add_argument('--dock-height-mm',type=float,help='use recorded SDK pitch and this front axle elevation above nominal')
parser.add_argument('--yaw-deg',type=float,help='rotate cube, then start at first touch along the approach axis')
args = parser.parse_args()
record = np.genfromtxt(root/'encoder.csv', names=True, delimiter=',')
v = params.values(params.load(model='vector_2'))
bx,_,bz = v['lift_base_position_mm']
t = np.arange(-.35,1.8,.001)
h = np.interp(t,record['seconds'],record['lift_mm'])
wrist = .001*np.column_stack([bx+np.sqrt(v['lift_arm_length_mm']**2-(h-bz)**2),np.zeros_like(h),h])
stem = 'printed-hold'
quat = np.tile([1.,0.,0.,0.],(len(t),1))
if args.dock_height_mm is not None:
    pitch_record=np.genfromtxt(root/'dock-pitch.csv',names=True,delimiter=',')
    pitch=np.interp(t,pitch_record['seconds'],pitch_record['pose_pitch_rad'])
    quat[:,0],quat[:,2]=np.cos(pitch/2),-np.sin(pitch/2)
    axle_z=v['wheel_front_radius_mm']*.001
    x,z=wrist[:,0].copy(),wrist[:,2]-axle_z
    wrist[:,0]=x*np.cos(pitch)-z*np.sin(pitch)
    wrist[:,2]=axle_z+x*np.sin(pitch)+z*np.cos(pitch)+args.dock_height_mm*.001
    stem = f'pitched-hold-{args.dock_height_mm:g}mm'
if args.yaw_deg is not None:
    stem += f'-yaw{args.yaw_deg:g}'
fig, axes = plt.subplots(1,2,figsize=(10,4),layout='constrained',sharey=True)
rows = []
for ax,(name,path) in zip(axes,[('Three boxes',root.parent/'unit3-cube-contact-replay/fixture.xml'),('Printed STL',args.fixture)]):
    model=mj.MjModel.from_xml_path(str(path))
    fork,cube = [model.body(n).id for n in ('fork','cube')]
    assert abs(model.opt.timestep-.001)<1e-9
    ids=np.flatnonzero((model.geom_bodyid==fork)&(model.geom_contype!=0))
    sizes=[]
    bottoms=[]
    init=mj.MjData(model)
    init.mocap_pos[0]=wrist[0]
    init.mocap_quat[0]=quat[0]
    mj.mj_forward(model,init)
    for g in ids:
        rotation=init.geom_xmat[g].reshape(3,3)[0]
        vertical=init.geom_xmat[g].reshape(3,3)[2]
        if model.geom_type[g] == mj.mjtGeom.mjGEOM_BOX:
            extent=abs(rotation)@model.geom_size[g]
            bottom=-abs(vertical)@model.geom_size[g]
        else:
            m=model.geom_dataid[g]
            vertices=model.mesh_vert[model.mesh_vertadr[m]:model.mesh_vertadr[m]+model.mesh_vertnum[m]]
            extent=np.max(vertices@rotation)
            bottom=np.min(vertices@vertical)
        bottoms.append(init.geom_xpos[g,2]+bottom)
        sizes.append(init.geom_xpos[g,0]+extent)
    for lateral in (-3,0,3):
        for gap in (-3,-1,0,1):
            d=mj.MjData(model)
            d.mocap_pos[0]=wrist[0]
            d.mocap_quat[0]=quat[0]
            d.joint('cube_root').qpos[:]=[max(sizes)+.02215+gap*.001,lateral*.001,.02215,1,0,0,0]
            def cross(c):
                return set(model.geom_bodyid[c.geom])=={fork,cube}
            if args.yaw_deg is not None:
                joint=d.joint('cube_root')
                half=np.deg2rad(args.yaw_deg)/2
                joint.qpos[3:] = [np.cos(half),0,0,np.sin(half)]
                # Find first external contact approached from clear space.
                clear=max(sizes)+.04
                for x in clear-np.arange(101)*.00025:
                    joint.qpos[0]=x
                    mj.mj_forward(model,d)
                    if any(cross(c) and c.dist<0 for c in d.contact):
                        break
                    clear=x
                else:
                    raise ValueError('No first contact in placement sweep')
                inside=x
                for _ in range(20):
                    mid=(inside+clear)/2
                    joint.qpos[0]=mid
                    mj.mj_forward(model,d)
                    if any(cross(c) and c.dist<0 for c in d.contact):
                        inside=mid
                    else:
                        clear=mid
                joint.qpos[0]=clear+gap*.001
            mj.mj_forward(model,d)
            penetration=max([0.,*[-c.dist*1000 for c in d.contact if cross(c)]])
            floor_gap_mm=min(bottoms)*1000
            if penetration>1e-5 or floor_gap_mm < -1e-5:
                rows.append([name,lateral,gap,penetration,floor_gap_mm,None,None,None,None,None])
                continue
            angles,heights,touch,load=[],[],[],[]
            for clock,pos,q in zip(t,wrist,quat):
                d.mocap_pos[0]=pos
                d.mocap_quat[0]=q
                mj.mj_step(model,d)
                angles.append(np.degrees(np.arccos(np.clip(d.xmat[cube,8],-1,1))))
                heights.append(d.xpos[cube,2]*1000)
                if 1.1 <= clock < 1.35:
                    forces=contact_load(model,d,cube,model.geom("table").id,fork)
                    load.append(sum(forces[1:3])/(model.body_mass[cube]*np.linalg.norm(model.opt.gravity)))
                touch.append(any(c.efc_address>=0 and cross(c) for c in d.contact))
            angles=np.array(angles)
            hold=(t>=1.1)&(t<1.35)
            rows.append([name,lateral,gap,penetration,floor_gap_mm,angles[t>=0].max(),angles[hold].max(),
                         np.mean(np.array(touch)[hold]),max(np.array(heights)[hold])-heights[np.searchsorted(t,0)],np.mean(load)])
            ax.plot(t,angles,lw=1,alpha=.7)
    ax.axvspan(1.1,1.35,color='orange',alpha=.2,label='Physical raised interval')
    ax.set(title=name,xlabel='Seconds from lift command',xlim=(0,1.65))
    ax.legend(fontsize=8)
axes[0].set_ylabel('Simulated cube inclination (degrees)')
fig.savefig(root/f'{stem}.png',dpi=165)
with (root/f'{stem}.csv').open('w') as f:
    w=csv.writer(f)
    w.writerow('fork lateral_mm envelope_gap_mm initial_penetration_mm initial_fork_floor_clearance_mm peak_deg raised_peak_deg raised_contact_fraction raised_max_rise_mm mean_upward_fork_force_weights'.split())
    w.writerows(rows)

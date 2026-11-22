"""Restart trial 1 at the raised exposure and predict the subsequent lowering frame."""
import csv
import sys

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from evaluate import run, repo, here
from front_edge import origin, basis
from optical_clearance import rows, reconstruct
sys.path.insert(0,str(repo))

times = rows['sec']
start = int(np.argmin(abs(times-5.498)))
end = int(np.argmin(abs(times-5.998)))
results = []
fig,axes = plt.subplots(2,2,figsize=(9,6),layout='constrained')
def tilt(q):
    return np.degrees(np.arccos(np.clip(Rotation.from_quat(q[3:],scalar_first=True).as_matrix()[2,2],-1,1)))

for shelf,(ax,angle_ax) in zip((False,True),axes.T):
    poses = reconstruct(-1.499,473.11,shelf)
    def pose(i):
        rot,delta,_ = poses[i]
        return np.r_[(origin+basis.apply(delta))/1000,(basis*rot).as_quat(scalar_first=True)]
    initial,target = pose(start),pose(end)
    ax.plot([times[start],times[end]], [1000*p[2] for p in (initial,target)],'ko',label='Optical centers')
    angle_ax.plot([times[start],times[end]],[tilt(p) for p in (initial,target)],'ko')
    for velocity in ('rest','preceding secant','cleared at rest'):
        trace=[]; gaps=[]; shifts=[]
        def inspect(m,d,t,lift):
            if t<times[start] or t>times[end]+.002:
                return
            joint=d.joint('cube_root')
            if not trace:
                joint.qpos[:]=initial
                joint.qvel[:]=0
                if velocity=='preceding secant':
                    q0=d.qpos.copy()
                    q0[m.joint('cube_root').qposadr[0]:m.joint('cube_root').qposadr[0]+7]=pose(start-1)
                    vel=np.zeros(m.nv)
                    mj.mj_differentiatePos(m,vel,times[start]-times[start-1],q0,d.qpos)
                    idx=m.joint('cube_root').dofadr[0]
                    joint.qvel[:]=vel[idx:idx+6]
                d.qacc_warmstart[:]=0
                mj.mj_forward(m,d)
                cube=m.body('cube').id
                shifts.append(np.zeros(3))
                if velocity=='cleared at rest':
                    # Search increasing translation norm on a 1 mm grid, holding attitude fixed.
                    choices=np.array(np.meshgrid(*([np.arange(-8,9)]*3))).reshape(3,-1).T
                    for offset in choices[np.argsort(np.sum(choices**2,axis=1),kind='stable')]:
                        joint.qpos[:3]=initial[:3]+offset/1000
                        mj.mj_forward(m,d)
                        if all(c.dist>=-1e-8 for c in d.contact if cube in m.geom_bodyid[c.geom]):
                            shifts[0]=offset
                            break
                    else:
                        raise ValueError('No clear restart within search grid')
                gaps.extend(c.dist*1000 for c in d.contact if cube in m.geom_bodyid[c.geom])
            trace.append((t,joint.qpos.copy()))
        run((1,'stl','overhead','weld',0,0,0),inspect,observer_stride=1,rails=True)
        t,p=min(trace,key=lambda sample:abs(sample[0]-times[end]))
        error=(p[:3]-target[:3])*1000
        attitude=(Rotation.from_quat(target[3:],scalar_first=True).inv()*Rotation.from_quat(p[3:],scalar_first=True)).magnitude()
        results.append(dict(shelf=shelf,velocity=velocity,initial_overlap_mm=max([0]+[-g for g in gaps]),
                            restart_dx_mm=shifts[0][0],restart_dy_mm=shifts[0][1],restart_dz_mm=shifts[0][2],
                            predicted_z_mm=p[2]*1000,observed_z_mm=target[2]*1000,
                            forward_error_mm=error[0],left_error_mm=error[1],height_error_mm=error[2],
                            position_error_mm=np.linalg.norm(error),rotation_error_deg=np.degrees(attitude)))
        angle_ax.plot([t for t,_ in trace],[tilt(p) for _,p in trace])
        ax.plot([t for t,_ in trace],[p[2]*1000 for _,p in trace],label=velocity)
    ax.set(title='Shelf-constrained pose' if shelf else 'Free optical pose',xlabel='Seconds after approach',ylabel='Cube center height (mm)')
    angle_ax.set(xlabel='Seconds after approach',ylabel='Cube top tilt (degrees)')
axes[0,0].legend(fontsize=8)
fig.savefig(here/'raised-restart.png',dpi=160)
with (here/'raised-restart.csv').open('w',newline='') as stream:
    writer=csv.DictWriter(stream,fieldnames=results[0].keys())
    writer.writeheader()
    writer.writerows(results)

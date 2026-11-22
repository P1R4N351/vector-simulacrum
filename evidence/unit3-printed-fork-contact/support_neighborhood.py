"""Hold the raised wrist still and release translated copies of the observed cube."""
import csv
from itertools import product
import cv2
import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from adjacent_face import crop_offset, checks, edge, folder, measure, project_face, residual_segment
from evaluate import here, run
from front_edge import basis, origin
from optical_clearance import rows, quads, reconstruct, marker

stamp,t0,a,b = checks[0]
image = cv2.imread(str(folder / (stamp+'.png')))
measured = measure(image,a,b)
reference = np.median(quads[rows['sec']<0],axis=0)
rot,delta,_ = reconstruct(-1.499,473.11,False)[np.argmin(abs(rows['sec']-t0))]
points = rot.apply(np.linspace(*edge,25))+delta
records = []

def survey(m,d,t,lift):
    if t<t0 or records:
        return
    initial = d.qpos.copy()
    sim = mj.MjData(m)
    cube = m.body('cube').id
    for dx,dy,dz in product(range(-8,9,2),range(-8,9,2),range(4,13,2)):
        shift = np.array([dx,dy,dz])
        mj.mj_resetData(m,sim)
        sim.mocap_pos[:] = d.mocap_pos
        sim.mocap_quat[:] = d.mocap_quat
        sim.qpos[:] = initial
        joint = sim.joint('cube_root')
        joint.qpos[:] = np.r_[(origin+basis.apply(delta)+shift)/1000,
                              (basis*rot).as_quat(scalar_first=True)]
        mj.mj_forward(m,sim)
        if any(c.dist < -1e-8 for c in sim.contact if cube in m.geom_bodyid[c.geom]):
            continue
        line = project_face(reference,-1.499,473.11,points+basis.inv().apply(shift))
        miss = np.median(np.min([np.linalg.norm(residual_segment(measured,line[j:j+2]),axis=1)
                                 for j in range(len(line)-1)],axis=0))
        top = project_face(reference,-1.499,473.11,rot.apply(marker)+delta+basis.inv().apply(shift))
        top_rms = np.sqrt(np.mean(np.sum((top + crop_offset - quads[np.argmin(abs(rows['sec']-t0))])**2,axis=1)))
        support = []
        for _ in range(round(.25/m.opt.timestep)):
            mj.mj_step(m,sim)
            force = 0.
            for i,c in enumerate(sim.contact):
                bodies = m.geom_bodyid[c.geom]
                if cube not in bodies or 0 in bodies:
                    continue
                wrench = np.zeros(6)
                mj.mj_contactForce(m,sim,i,wrench)
                force += (1 if bodies[1]==cube else -1)*(wrench[:3]@c.frame.reshape(3,3))[2]
            support.append(force)
        tilt = np.degrees(np.arccos(np.clip(1-2*np.sum(joint.qpos[4:6]**2),-1,1)))
        records.append((dx,dy,dz,miss,tilt,np.mean(support[-50:]),top_rms))
run((1,'stl','overhead','weld',0,0,0),survey,observer_stride=1,rails=True)
with (here/'support-neighborhood.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('dx_mm','dy_mm','dz_mm','raised_edge_error_px','tilt_at_250ms_deg','last_50ms_robot_vertical_N','top_marker_rms_px'))
    w.writerows(records)
r=np.asarray(records)
fig,ax=plt.subplots(figsize=(7,4),layout='constrained')
dots=ax.scatter(r[:,3],r[:,4],c=r[:,6],s=18,cmap='viridis')
fig.colorbar(dots,label='Raised top-marker RMS error (pixels)')
ax.set(xlabel='Raised border error (pixels)',ylabel='Cube tilt after 250 ms (°)',title='Fixed raised wrist · collision-free releases')
fig.savefig(here/'support-neighborhood.png',dpi=160)
print(len(r),r[np.argmax(r[:,4])],r[np.argmin(r[:,3])],min(r[:,5]),max(r[:,5]))

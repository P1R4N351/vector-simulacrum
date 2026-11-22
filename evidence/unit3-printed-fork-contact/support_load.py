"""Resolve shelf, fork and link loads on the cube after a cleared restart."""
import csv
import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation
from evaluate import here, run
from front_edge import origin, basis
from optical_clearance import rows, reconstruct

groups = ('shelf', 'fork', 'links')
starts = np.genfromtxt(here / 'raised-restart.csv', delimiter=',', names=True, dtype=None, encoding='utf8')
index = np.argmin(abs(rows['sec'] - 5.498))
fig, axes = plt.subplots(3, 2, figsize=(9, 7), sharex=True, layout='constrained')
with (here/'support-load.csv').open('w', newline='') as stream:
    out = csv.writer(stream, lineterminator='\n')
    out.writerow(('start','elapsed_s','encoder_lift_mm','tilt_deg',*(g+'_vertical_N' for g in groups)))
    for shelf, ax in zip((False, True), axes.T):
        restart = next(r for r in starts if r['shelf']==shelf and r['velocity']=='cleared at rest')
        offset = np.array([restart['restart_d'+k+'_mm'] for k in 'xyz'])
        rot, delta, _ = reconstruct(-1.499, 473.11, shelf)[index]
        samples = []
        def inspect(m,d,t,lift):
            elapsed = t-rows['sec'][index]
            if not 0 <= elapsed <= .51:
                return
            cube = m.body('cube').id
            joint = d.joint('cube_root')
            if not samples:
                joint.qpos[:] = np.r_[(origin+basis.apply(delta)+offset)/1000,
                                      (basis*rot).as_quat(scalar_first=True)]
                joint.qvel[:] = 0
                d.qacc_warmstart[:] = 0
            mj.mj_forward(m,d)
            vertical = dict.fromkeys(groups,0.)
            for i,c in enumerate(d.contact):
                bodies = m.geom_bodyid[c.geom]
                if cube not in bodies:
                    continue
                other = m.body(bodies[0] if bodies[1]==cube else bodies[1]).name
                group = {'world':'shelf','fork':'fork','rail0':'links','rail1':'links'}[other]
                force = np.zeros(6)
                mj.mj_contactForce(m,d,i,force)
                sign = 1 if bodies[1]==cube else -1
                vertical[group] += sign*(force[:3] @ c.frame.reshape(3,3))[2]
            tilt = np.degrees(np.arccos(np.clip(1-2*np.sum(joint.qpos[4:6]**2),-1,1)))
            row = (elapsed,lift,tilt,*vertical.values())
            samples.append(row)
            out.writerow((str(shelf),*row))
        run((1,'stl','overhead','weld',0,0,0), inspect, observer_stride=1, rails=True)
        t,lift,angle,*loads = np.array(samples).T
        ax[0].set_title('Shelf-constrained start' if shelf else 'Free optical start')
        ax[0].plot(t,lift)
        ax[1].plot(t,angle)
        for g,load in zip(groups,loads):
            ax[2].plot(t,load,label=g)
        ax[2].set_xlabel('Seconds since raised frame')
        print(shelf,'last tilt >5',t[angle>5][-1],'lift then',lift[angle>5][-1],
              'lift range',min(lift),max(lift),'last upward support >1 mN',
              [max(t[load>.001],default=None) for load in loads])
for ax,label in zip(axes[:,0],('Encoder lift (mm)','Simulated tilt (°)','Vertical force (N)')):
    ax.set_ylabel(label)
axes[2,1].legend()
fig.savefig(here/'support-load.png',dpi=160)

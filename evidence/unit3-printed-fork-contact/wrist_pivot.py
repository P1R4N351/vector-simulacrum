"""Minimum change of a cube point in wrist coordinates through the raised exposures."""
import csv

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import lsq_linear
from optical_clearance import here, reconstruct, rows, run, quads
from recovered_marker import recovered

times = rows['sec']
extra = recovered()
bias = quads[np.argmin(abs(times-extra[1][0]))] - extra[1][1]
times = np.r_[times, extra[0][0]]
corners = np.concatenate([quads, [extra[0][1]+bias]])
indices = [int(np.argmin(abs(times-t))) for t in (4.985, 5.264, 5.498)]
shifts = np.arange(-200,201,10)
snapshots = {}
anchor = {}
def observe(model, data, t, lift):
    if not anchor:
        body = data.body('cube')
        anchor.update(R=body.xmat.reshape(3,3).copy(), p=1000*body.xpos.copy())
    for i in indices:
        shift = round((t-times[i])*1000)
        if -200 <= shift <= 200 and shift % 10 == 0:
            fork = data.body('fork')
            snapshots[i,shift] = (fork.xmat.reshape(3,3).T.copy(), 1000*fork.xpos.copy())

# Direct wrist positioning makes the sampled trajectory independent of collision geometry.
run((1,'boxes','overhead','mocap',0,0,0), observe, observer_stride=1)
records = []
for lens,k,f in (('See3CAM',-1.499,473.11), ('Pinhole',0,700)):
    for shelf in (False, True):
        optical = reconstruct(k,f,shelf,corners,times)
        for shift in shifts:
            transforms = []
            for i in indices:
                rot,delta,_ = optical[i]
                R,p = snapshots[i,shift]
                transforms.append((R @ anchor['R'] @ rot.as_matrix(),
                                   R @ (anchor['p'] + anchor['R'] @ delta - p)))
            A,u = transforms[0]
            matrices = np.vstack([B-A for B,_ in transforms[1:]])
            shifts_to_match = np.concatenate([u-v for _,v in transforms[1:]])
            fit = lsq_linear(matrices,shifts_to_match,bounds=(-22.15,22.15))
            errors = np.linalg.norm(fit.fun.reshape(2,3), axis=1)
            records.append((lens,shelf,np.sqrt(np.mean(errors**2)),*errors,*fit.x,shift))
with (here/'wrist-pivot.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['lens','shelf','rms_mm','middle_mm','end_mm','cube_x_mm','cube_y_mm','cube_z_mm','wrist_time_shift_ms'])
    w.writerows(records)
fig,ax=plt.subplots(figsize=(7,4),layout='constrained')
for lens,shelf in dict.fromkeys(r[:2] for r in records):
    group=[r for r in records if r[:2]==(lens,shelf)]
    ax.plot([r[-1] for r in group],[r[2] for r in group],'.-',label=lens+(' shelf' if shelf else ' free'))
ax.set(xlabel='Wrist time shift (ms)',ylabel='Minimum RMS displacement in fork frame (mm)',title='Same cube point against the moving wrist · trial 1')
ax.legend()
fig.savefig(here/'wrist-pivot.png',dpi=160)

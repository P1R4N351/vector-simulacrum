"""Compare the cube's received specific-force magnitude through the failed hold."""
import csv
from pathlib import Path

import mujoco
import matplotlib.pyplot as plt
import numpy as np

from face_contact import start_raise

root = Path(__file__).resolve().parent
physical = np.loadtxt(root/'cube.csv', delimiter=',', skiprows=1).reshape(-1,3,5)
assert np.all(physical[:,:,1]==[0,1,2])
fig,axes = plt.subplots(2,1,figsize=(8,5.5),sharex=True,layout='constrained')
optical = np.loadtxt(root/'glyph-track.csv', delimiter=',', skiprows=1)
for i,view in enumerate('AB'):
    rise = optical[:,5+6*i]-optical[:,2+6*i]
    axes[0].plot(optical[:,0],rise-np.median(rise[optical[:,0]<0]),'.-',label=f'Camera {view}')
axes[0].set_ylabel('Glyph rise relative to hub (px)')
series=[('Physical', physical[:,0,0], physical[:,:,2:]/8192)]
for gap in (0,1,2):
    sim,_ = start_raise(gap)
    start = sim.data.time
    sim.advance(2)
    packets = [(t-start,p) for t,p in sim.cube.messages if t>start]
    xyz = np.stack([np.frombuffer(p[2:],dtype='<i2').reshape(3,3) for _,p in packets])/sim.v['cube_accel_counts_per_g']
    series.append((f'{gap} mm gap',np.array([t for t,_ in packets]),xyz))
# Known free drop checks that the simulated packet chain can expose an event.
# Ten mm is a sensitivity input, independent of any camera-to-mm conversion.
for shift in (0,.01,.02):
    sim,_ = start_raise(2)
    start = sim.data.time
    sim.advance(1.35+shift)
    joint = sim.data.joint("cube_root")
    joint.qpos[0] += .15  # clear the fork for a shelf-only drop
    joint.qpos[2] += .010
    joint.qvel[:] = 0
    mujoco.mj_forward(sim.model,sim.data)
    sim.advance(.4)
    packets=[(t-start,p) for t,p in sim.cube.messages if t>start]
    xyz=np.stack([np.frombuffer(p[2:],dtype='<i2').reshape(3,3) for _,p in packets])/sim.v['cube_accel_counts_per_g']
    series.append((f'10 mm drop +{round(shift*1000)} ms',np.array([t for t,_ in packets]),xyz))
rows=[]
for label,t,xyz in series:
    norm=np.linalg.norm(xyz,axis=2)
    for lo,hi in [(1.35,1.45),(1.25,1.55)]:
        sel=(t>=lo)&(t<=hi)
        a=norm[sel]
        rows.append([label,lo,hi,len(a),a.min(),a.max(),int((a<.6).sum()),int((a>2).sum())])
    if label.startswith('10 mm drop') and not label.endswith('+0 ms'):
        continue
    mask=(t>=1.13)&(t<=1.76)
    axes[1].plot(np.repeat(t[mask],3),norm[mask].ravel(),'.',ms=5 if label=='Physical' else 3,
                 zorder=5 if label=='Physical' else 2,label=label)
axes[1].axvspan(1.428972,1.698582,alpha=.08,color='gray',label='Physical packet gap')
axes[1].set(ylabel='Specific-force magnitude (g)',xlabel='Seconds from lift command',xlim=(1.13,1.76))
for ax in axes: ax.legend(ncol=3,fontsize=8)
fig.savefig(root/'release-accel.png',dpi=160)
with (root/'release-accel.csv').open('w') as f:
    w=csv.writer(f)
    w.writerow('source start_s end_s packets min_g max_g slots_below_0.6g slots_above_2g'.split())
    w.writerows(rows)

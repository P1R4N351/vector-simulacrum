"""Predict SDK pose on a successful mount from the last precontact charger observation."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from release_optics import states

here = Path(__file__).resolve().parent
fig, axes = plt.subplots(2, 5, figsize=(13, 5), sharex=True, sharey='row', layout='constrained')
with (here/'mount-target-inputs.csv').open() as inputs, (here/'mount-target-scores.csv').open('w') as out:
    writer = csv.writer(out, lineterminator='\n')
    writer.writerow(['session','last_observation_age_s','first_on_position_error_mm','first_on_yaw_error_deg',
                     'rest_position_error_mm','rest_yaw_error_deg'])
    for col, obs in enumerate(csv.DictReader(inputs)):
        name = obs['session']
        records, release = states(name)
        t = np.array([float(r['receipt_s']) for r in records])
        on = np.array([bool(int(r['status']) & 4096) for r in records])
        k = next(i for i in range(release + 1,len(t)) if t[i]-t[release]>3 and on[i] and not on[i-1])
        t -= t[k]
        a,b,c,d = [float(obs['q'+str(j)]) for j in range(4)]
        # Project the dock's local +X axis into the recorded origin.
        axis = np.array([1-2*(c*c+d*d),2*(a*d+b*c)])
        target = [float(obs['charger_x']),float(obs['charger_y'])] + 30*axis
        yaw = np.arctan2(-axis[1],-axis[0])
        xy = np.array([[float(r['x_mm']),float(r['y_mm'])] for r in records])
        err = np.linalg.norm(xy-target,axis=1)
        angle = np.array([float(r['yaw_rad']) for r in records])-yaw
        angle = np.rad2deg(np.arctan2(np.sin(angle),np.cos(angle)))
        window = (t>=-.45)&(t<=.65)
        rest = (t>=.3)&(t<=.65)
        writer.writerow([name,obs['age_s'],err[k],angle[k],np.median(err[rest]),np.median(angle[rest])])
        for ax, values in zip(axes[:,col],[err,angle]):
            ax.axhline(0,color='#888',lw=.8)
            ax.axvspan(t[k-1],0,color='silver',alpha=.4)
            ax.plot(t[window],values[window],'o-',ms=3,color='#196882')
            ax.scatter(0,values[k],color='#c75420',zorder=5)
        axes[0,col].set_title('Return '+name)
        axes[1,col].set_xlabel('Seconds from contact')
axes[0,0].set_ylabel('Distance to target (mm)')
axes[1,0].set_ylabel('Yaw minus target (°)')
fig.suptitle('SDK pose against the target predicted from the last charger observation')
fig.savefig(here/'mount-target.png',dpi=150)

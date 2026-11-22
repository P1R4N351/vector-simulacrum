"""Test pre-motion range adjustments against independently observed shelf pushes."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent/'unit3-printed-fork-contact'))
import evaluate

ranges = np.genfromtxt(here/'initial-depth.csv', names=True, delimiter=',')
residual = ranges['observed_median_mm'] - ranges['original_median_mm']
sensitivity = (ranges['original_median_mm']-ranges['closer_median_mm'])/2
tracks = np.genfromtxt(here.parent/'unit3-paired-cube-push/traces.csv',names=True,delimiter=',',dtype=None,encoding='utf8')
def onset(t, xy):
    displacement=np.linalg.norm(xy,axis=1)
    ids=np.flatnonzero((t[:-1]>=0)&(displacement[:-1]>=2)&(displacement[1:]>=2))
    return float(t[ids[0]]) if len(ids) else None
rows = []
for i in range(len(ranges)):
    obs=tracks[(tracks['trial']==i+1)&(tracks['camera']=='frames')]
    obs_time=obs['seconds']
    real_onset=onset(obs_time,np.column_stack([obs['forward_mm'],obs['left_mm']]))
    bias = np.median(np.delete(residual,i))
    for method,offset in [('camera',0.),('range',residual[i]/sensitivity[i]),
                          ('range with bias',(residual[i]-bias)/sensitivity[i])]:
        contact_times = []
        def check(model,data,t,lift):
            if contact_times:
                return
            bodies = model.geom_bodyid[data.contact.geom]
            pair = [model.body('fork').id, model.body('cube').id]
            if np.any(np.all(np.sort(bodies,axis=1)==sorted(pair),axis=1)):
                contact_times.append(t)
        score, trace = evaluate.run((i+1,'stl','overhead','weld',offset,0,0),check)
        path=np.asarray(trace)
        sampled=np.column_stack([np.interp(obs_time,path[:,0]-1.5,path[:,k]) for k in (1,2)])
        rows.append(dict(trial=i+1, method=method, depth_change_mm=offset,
                         other_trials_bias_mm=bias, first_contact_s=next(iter(contact_times),None),
                         camera_motion_s=real_onset,sim_motion_s=onset(obs_time,sampled),forward_mm=score[8],left_mm=score[9],endpoint_error_mm=score[12]))
with (here/'range-contact-transfer.csv').open('w',newline='') as stream:
    w=csv.DictWriter(stream,fieldnames=rows[0].keys()); w.writeheader(); w.writerows(rows)
fig,axes=plt.subplots(1,2,figsize=(8,3.5),layout='constrained')
for ax,metric,title in zip(axes,['depth_change_mm','endpoint_error_mm'],['Initial depth change (mm)','Endpoint vector error (mm)']):
    for method in dict.fromkeys(r['method'] for r in rows):
        selected=[r for r in rows if r['method']==method]
        ax.plot(range(1,6),[r[metric] for r in selected],'o-',label=method)
    ax.set(xlabel='Physical push trial',ylabel=title,xticks=range(1,6))
axes[1].legend(fontsize=8)
fig.savefig(here/'range-contact-transfer.png',dpi=160)

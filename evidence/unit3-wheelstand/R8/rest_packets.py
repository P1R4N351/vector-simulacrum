"""Score within-packet differences and static coherence on two observed cube faces."""
import csv
import gzip
import json
from pathlib import Path
import subprocess
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
def packets(name,start=-np.inf,end=np.inf):
    with gzip.open(here/name,'rt') as f:
        r=np.genfromtxt(f,delimiter=',',names=True)
    r=r[(r['sec']>=start)&(r['sec']<end)]
    assert np.array_equal(r['slot'],np.tile([0,1,2],len(r)//3))
    return np.column_stack([r[k] for k in 'xyz']).reshape(-1,3,3)/8192

with (here/'actions.csv').open() as f:
    zero=next(float(r['sec']) for r in csv.DictReader(f) if r['kind']=='begin' and 'roll' in r['action'])
obs={'Before roll':packets('cube.csv.gz',zero-.9,zero),'Settled':packets('settled.csv.gz')}
def statistics(p):
    d=np.diff(p,axis=1)
    short=np.sqrt(np.mean(d*d,axis=(0,1))/2)*1000
    long=np.std(p.mean(axis=1),axis=0)*1000
    return np.stack([short,long])

values=json.loads((here.parents[2]/'web/public/robot/vector2/model.json').read_text())['values']
cases=[]
for label,p in obs.items():
    force=(p.mean(axis=(0,1))-np.array(values['cube_accel_bias_mg'])/1000)/values['cube_accel_scale']
    force/=np.linalg.norm(force)
    cases.extend(dict(name=label,seed=k,packets=len(p),force=force.tolist()) for k in range(1,25))
out=subprocess.check_output(['node',str(here/'rest_packets.mjs')],input=json.dumps(cases),text=True)
runs={}
for line in out.splitlines():
    label,seed,slots=json.loads(line)
    runs.setdefault((label,seed),[]).append(slots)
fig,axes=plt.subplots(2,2,figsize=(8,6),layout='constrained',sharey='row')
scores=[]
for col,(label,p) in enumerate(obs.items()):
    real=statistics(p)
    sims=np.stack([statistics(np.array(runs[label,k])/8192) for k in range(1,25)])
    lo,mid,hi=np.percentile(sims,[2.5,50,97.5],axis=0)
    for j,metric in enumerate(['Within-packet noise','Packet-mean scatter']):
        ax=axes[j,col]
        ax.errorbar(np.arange(3)+.08,mid[j],yerr=[mid[j]-lo[j],hi[j]-mid[j]],fmt='o',capsize=4,label='Simulation')
        ax.scatter(np.arange(3)-.08,real[j],label='Physical',color='#b7592c')
        ax.set_xticks(range(3),list('XYZ'));ax.set_ylim(bottom=0)
        ax.set_title(f'{label} · {metric}');ax.set_ylabel('mg')
        for i,k in enumerate('XYZ'):
            scores.append([label,len(p),metric,k,real[j,i],mid[j,i],lo[j,i],hi[j,i]])
axes[0,0].legend(fontsize=8)
fig.savefig(here/'rest-packets.png',dpi=160)
with (here/'rest-packets.csv').open('w') as f:
    w=csv.writer(f);w.writerow('window packets metric axis physical_mg simulated_median_mg seed_lo_mg seed_hi_mg'.split());w.writerows(scores)

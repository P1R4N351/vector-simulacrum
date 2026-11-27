"""Register the last physical cube packets between the two measured rest vectors."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
def load(filename):
    r=np.genfromtxt(here/filename,names=True,delimiter=',')
    assert np.array_equal(r['slot'],np.tile([0,1,2],len(r)//3))
    return r['sec'][::3],np.stack([r[k] for k in 'xyz'],axis=1).reshape(-1,3,3)/8192

with (here/'actions.csv').open() as f:
    zero=next(float(row['sec']) for row in csv.DictReader(f) if row['action']=='roll cube' and row['kind']=='begin')
t,p=load('cube.csv.gz');t-=zero
_,end=load('settled.csv.gz')
before=p[(t>=-.9)&(t<0)]
def unit(x):
    return x/np.linalg.norm(x)
a,b=unit(before.mean(axis=(0,1))),unit(end.mean(axis=(0,1)))
u=unit(b-(a@b)*a)
def direction(v):
    return np.degrees(np.arctan2(v@u,v@a))
v=p.mean(axis=1)
angles=direction(v)
def change(scores,n):
    return np.median(scores[-n:])-np.median(scores[:n])
late=(t>=4.97)
fig,(ax,bx)=plt.subplots(2,1,figsize=(8,5),layout='constrained',sharex=True)
ax.plot(t[t>=4.35],angles[t>=4.35],'.-',label='Physical packet means')
ax.axhline(direction(b),ls=':',color='gray',label='Later settled face')
ax.set_ylabel('Force direction (degrees)');ax.legend()
ref=np.genfromtxt(here/'optical-settle.csv',names=True,delimiter=',')
ct=ref['camera_s'];match=ref['exposed_final_similarity']
persistent=ct[np.flatnonzero(match<.98)[-1]+1]
clock_min,clock_max=59.754,59.875
keep=(ct>64)&(ct<65.8)
bx.plot(ct[keep]-(clock_min+clock_max)/2,match[keep],'.-',color='#276776')
bx.fill_betweenx([0,1],persistent-clock_max,persistent-clock_min,color='#db8c42',alpha=.25,label='Persistent final appearance')
bx.axvline(t[-1],color='.4',ls=':',label='Last cube packet')
bx.set_ylabel('Final-pose patch similarity');bx.set_xlabel('Seconds after roll command');bx.legend(fontsize=8)
ax.set_xlim(4.35,5.95)
fig.savefig(here/'final-direction.png',dpi=165)
# Slide a window of identical packet count across the later resting capture.
nlate=sum(late); rest=direction(end.mean(axis=1))
with (here/'final-direction.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['end_packets_per_group','early_median_deg','late_median_deg','change_deg','largest_abs_rest_change_deg'])
    for n in (2,3,4,5):
        row=[n,np.median(angles[late][:n]),np.median(angles[-n:]),change(angles[late],n),
             max(abs(change(rest[i:i+nlate],n)) for i in range(len(rest)-nlate+1))]
        w.writerow(row);print(row)
print('late packets',nlate,'last time, angle, norm',t[-1],angles[-1],np.linalg.norm(v[-1]))
print('time to persistent optical match',persistent-clock_max-t[-1],persistent-clock_min-t[-1])

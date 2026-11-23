"""Evaluate departure distance from the physical SDK flag bracket midpoint."""
from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt
p=Path(__file__).resolve().parent
runs=np.genfromtxt(p/'sweep.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
evidence=p.parent
opt=np.genfromtxt(evidence/'unit3-cube-range/release-optics.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
physical=dict(zip(opt['session'],opt['interpolated_midpoint_mm']))
h=np.genfromtxt(evidence/'unit3-w34-return/departure-hubs.csv',delimiter=',',names=True)
import gzip
with gzip.open(evidence/'unit3-w34-return/continuation.csv.gz','rt') as f: states=list(csv.DictReader(f))
k=next(i for i in range(1,len(states)) if int(states[i-1]['status'])&4096 and not int(states[i]['status'])&4096)
mid=(float(states[k-1]['receipt_s'])-float(states[k]['receipt_s']))/2
physical['W35']=np.mean([np.interp(mid,h['t_s'],h[key][0]-h[key]+h['static_x']-h['static_x'][0])*48/124 for key in ('front_x','rear_x')])
dist=lambda s:np.array([physical[n] for n in s['trial']])
offsets=np.unique(runs['offset_mm'])
def group(offset,names):
    return runs[(runs['offset_mm']==offset)&np.isin(runs['trial'],names)]
fit=list('ABC')
hold=[*'DEF','W35']
def mae(s):return np.mean(np.abs(s['travel_mm']-dist(s)))
best=min(offsets,key=lambda o:mae(group(o,fit)))
fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
for names,label in [(fit,'A–C fit'),(hold,'D–F and W35 held out')]:
    axes[0].plot(offsets,[mae(group(o,names)) for o in offsets],'.-',label=label)
    print(label,'offset',best,'MAE before/after',mae(group(0,names)),mae(group(best,names)))
for o,label in [(0,'Canonical seated start'),(best,f'{best} mm outward')]:
    s=group(o,fit+hold)
    axes[1].plot(s['trial'],s['travel_mm'],'.-',label=label)
    print(list(zip(s['trial'],dist(s),s['travel_mm'],s['release_s'])))
axes[0].axvline(best,ls=':',color='gray');axes[0].set(xlabel='Start shifted outward (mm)',ylabel='Mean absolute distance error (mm)')
axes[1].plot(s['trial'],dist(s),'kD',label='Camera at SDK bracket midpoint')
axes[1].set_ylabel('Travel to electrical release (mm)')
for ax in axes:ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.savefig(p/'seating.png',dpi=160)
print('W35 SDK pitch deg',float(states[k-8]['pitch_rad'])*180/np.pi)

"""Identify interrupted descents before the first in-position flag on SDK packet times."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

def read(path):
    with Path(path).open() as f:
        return list(csv.DictReader(f))

def score(t,h,flags,start,target):
    first=np.flatnonzero((t>=start)&(flags==0))[0]
    end=first+np.flatnonzero(flags[first:]==1)[0]
    trough=first+np.argmin(h[first:end+1])
    pre=np.median(h[(t>=start-.2)&(t<start)])
    candidates=[]
    i=first
    while i<trough:
        j=i
        while j<trough and abs(h[j+1]-h[i])<.05:
            j+=1
        if j-i>=2 and pre-h[i]>=1 and h[i]-target>=1 and h[i]-h[trough]>=1:
            candidates.append((t[j]-t[i],i,j))
        i=j+1
    return pre,end,max(candidates) if candidates else None

def main():
    rows=read('sdk.csv')
    sim=read('replay.csv')
    states=[r for r in rows if r['event']=='state']
    t,h,flags=(np.array([float(x[k]) for x in states]) for k in ('relative_s','lift_mm','in_position'))
    st,sh,sf=(np.array([float(x[k]) for x in sim]) for k in ('s','lift_mm','in_position'))
    commands=[r for r in rows if r['event']=='command' and r['target'] in ('48','56')]
    fig,axes=plt.subplots(2,4,figsize=(12,6),sharex=True,sharey=True,layout='constrained')
    with open('scores.csv','w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(('trial','target_mm','source','start_mm','pause_mm','pause_s','flag_s'))
        for n,(r,ax) in enumerate(zip(commands,axes.flat),1):
            start=float(r['relative_s'])
            target=float(r['target'])
            for clock,height,flag,source in ((t,h,flags,'physical'),(st,sh,sf,'canonical')):
                pre,end,pause=score(clock,height,flag,start,target)
                section=(clock>=start-.12)&(clock<=clock[end]+.12)
                ax.plot(clock[section]-start,height[section],label=source)
                if pause:
                    dur,i,j=pause
                    ax.axvspan(clock[i]-start,clock[j]-start,alpha=.2,color='green')
                w.writerow((n,target,source,pre,height[pause[1]] if pause else '',pause[0] if pause else '',clock[end]-start))
            ax.axhline(target,ls=':',color='gray')
            ax.set_title(f'Trial {n}, {target:g} mm')
    axes[0,0].legend()
    fig.supxlabel('Seconds after lower command')
    fig.supylabel('Lift height (mm)')
    fig.savefig('lowers.png',dpi=160)

if __name__=='__main__':
    main()

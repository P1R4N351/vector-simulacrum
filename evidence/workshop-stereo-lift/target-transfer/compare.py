"""Overlay matched physical commands and apply the frozen workshop pause score."""
from pathlib import Path
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt

prior=Path(__file__).resolve().parent.parent/'target-crossover'
sys.path.insert(0,str(prior))
from analyze import read,score


def physical(folder):
    data=read(folder/'sdk.csv')
    states=[r for r in data if r['event']=='state']
    t,h,f=(np.array([float(r[k]) for r in states]) for k in ('relative_s','lift_mm','in_position'))
    commands=[r for r in data if r['event']=='command' and r['target'] in ('48','56')]
    return t,h,f,commands


def main():
    original=physical(prior)
    transfer=physical(Path(__file__).parent)
    expect=[48,56,56,48,56,48,48,56]
    fig,axes=plt.subplots(2,4,figsize=(12,6),sharex=True,sharey=True,layout='constrained')
    with open('transfer.csv','w') as out:
        w=csv.writer(out,lineterminator='\n')
        w.writerow(('trial','target_mm','robot','pre_mm','minimum_mm','recovery_mm','pause_mm','pause_s','flag_s'))
        for (t,h,f,commands),label,color in ((original,'First generation','tab:blue'),(transfer,'Second generation','tab:orange')):
            assert [int(r['target']) for r in commands]==expect
            for n,(ax,r,target) in enumerate(zip(axes.flat,commands,expect),1):
                start=float(r['relative_s'])
                pre,end,pause=score(t,h,f,start,target)
                mask=(t>=start-.1)&(t<=t[end]+.1)
                low=np.min(h[(t>=start)&(t<=t[end])])
                ax.plot(t[mask]-start,h[mask],label=label,color=color)
                if pause:
                    dur,i,j=pause
                    ax.plot(t[i:j+1]-start,h[i:j+1],lw=5,color=color,alpha=.5)
                w.writerow((n,target,label,pre,low,h[end]-low,h[pause[1]] if pause else '',pause[0] if pause else '',t[end]-start))
                ax.set_title(f'{n}. Target {target} mm')
    for ax,target in zip(axes.flat,expect):
        ax.axhline(target,color='gray',ls=':')
    axes[0,0].legend(fontsize=8)
    fig.supxlabel('Seconds after lower command')
    fig.supylabel('Reported fork height (mm)')
    fig.savefig('transfer.png',dpi=160)

if __name__=='__main__':
    main()

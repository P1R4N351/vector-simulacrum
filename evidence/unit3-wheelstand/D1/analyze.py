"""Compare cube acceleration during 10–90% of recorded docked lift strokes."""
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(sys.argv[1]).resolve() if len(sys.argv)>1 else Path(__file__).resolve().parent


def main():
    state=np.genfromtxt(HERE/'state.csv.gz',names=True,delimiter=',')
    cube=np.genfromtxt(HERE/'cube.csv.gz',names=True,delimiter=',')
    mag=np.linalg.norm(np.column_stack([cube[k] for k in 'xyz']),axis=1)*1000/8192
    with (HERE/'actions.csv').open() as f:
        events=list(csv.DictReader(f))
    fig,axes=plt.subplots(2,2,figsize=(10,6),sharex=True,layout='constrained')
    with (HERE/'moves.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['condition','direction','repeat','travel_mm','rise_10_90_s','speed_mm_s','rest_mg','moving_mg','slots','tap_range','max_wheel_mm_s','active_wheel_mm_s','max_crossing_gap_s','timing_valid'])
        for event in events:
            parts=event['action'].split()
            if event['kind']!='begin' or parts[0] not in ('fast','slow','matched'): continue
            condition,direction,repeat=parts
            start=float(event['sec'])
            end=float(next(e['sec'] for e in events if e['action']==event['action'] and e['kind']=='end'))
            sel=(state['sec']>=start-.5)&(state['sec']<=end+.7)
            t=state['sec'][sel]-start
            h=state['lift'][sel]
            base=np.median(h[t<0])
            travel=np.median(h[t>end-start+.2])-base
            progress=(h-base)/travel
            def cross(level):
                j=np.flatnonzero((t>=0)&(progress>=level))[0]
                return np.interp(level,progress[j-1:j+1],t[j-1:j+1])
            lo,hi=cross(.1),cross(.9)
            ct=cube['sec']-start
            idle=(ct>=-.8)&(ct<-.1)
            active=(ct>=lo)&(ct<=hi)
            assert active.sum()>=6 and idle.sum()>=6
            wheels=max(np.abs(state[k][sel]).max() for k in ('left','right'))
            gap=np.max(np.diff(t)[(t[:-1]<=hi)&(t[1:]>=lo)])
            valid=gap<=.25
            active_states=sel & (state['sec']>=start+lo) & (state['sec']<=start+hi)
            active_wheel=max(np.abs(state[k][active_states]).max() for k in ('left','right'))
            w.writerow([condition,direction,repeat,travel,hi-lo if valid else '',.8*abs(travel)/(hi-lo) if valid else '',mag[idle].std(),mag[active].std() if valid else '',active.sum(),np.ptp(cube['tap'][active]),wheels,active_wheel,gap,int(valid)])
            if not valid: continue
            col=int(direction=='lower')
            color={'fast':'#b74224','slow':'#277698','matched':'#386439'}[condition]
            axes[0,col].plot(t,h,color=color,alpha=.7,label=condition if repeat=='1' else None)
            centers=np.arange(-.45,end-start+.6,.1)
            spreads=[mag[np.abs(ct-x)<.05].std() if (np.abs(ct-x)<.05).sum()>=6 else np.nan for x in centers]
            axes[1,col].plot(centers,spreads,color=color,alpha=.7)
    for ax,title in zip(axes[0],['Raise','Lower']):
        ax.set_title(title)
        ax.legend()
    for ax in axes[1]: ax.set_xlabel('Seconds from command')
    axes[0,0].set_ylabel('Lift encoder height (mm)')
    axes[1,0].set_ylabel('Cube acceleration SD (mg)')
    fig.savefig(HERE/'physical-profiles.png',dpi=150)


if __name__=='__main__':
    main()

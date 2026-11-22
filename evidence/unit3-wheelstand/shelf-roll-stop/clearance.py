"""Select clearance on R5, then score Sunday without refitting."""
import csv
from itertools import product
import matplotlib.pyplot as plt
import numpy as np
from impact_sensor import simulated, here

def score(angle, peak, gap):
    pairs = joint = total = 0
    for axis,sign,tick,frame in product(range(2),(-1,1),np.arange(10)/10,(0.,.5)):
        _,g = simulated(axis,sign,tick,frame,angle,gap)
        for offset in range(3):
            total += 1
            valid = [i for i in range(len(g)-2) if (i+offset)%3 != 2 and max(g[i:i+2]) < .05]
            pairs += bool(valid)
            joint += any(abs(max(g[i+2:i+11])-peak)<=.05*peak for i in valid)
    return pairs/total,joint/total

if __name__ == '__main__':
    gaps = (0,1,2,3,4,6,8,10)
    training = [score(31.5,4.0198,h) for h in gaps]
    # Prefer the smallest clearance in the event of a tie.
    best = max(range(len(gaps)), key=lambda i: training[i][1])
    print('selected',gaps[best],training)
    fig,ax=plt.subplots(layout='constrained')
    rows = [('R5',h,*v) for h,v in zip(gaps,training)]
    for j,label in enumerate(('Low-force pair','Pair and matching impact')):
        ax.plot(gaps,[100*v[j] for v in training],'.-',label='R5: '+label)
    for h in sorted({0,gaps[best]}):
        v=score(15.76,3.82534,h)
        print('holdout',h,v)
        rows.append(('S22B',h,*v))
        ax.scatter(h,100*v[1],marker='x',s=80,color='black',label='Sunday: pair and matching impact' if h==0 else None)
    ax.set(xlabel='Initial clearance beneath lowest corner (mm)',ylabel='Cases meeting criterion (%)',title='Does an unsupported release reproduce both sensor features?')
    ax.legend();fig.savefig(here/'clearance.png',dpi=150)
    with (here/'clearance.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['record','gap_mm','pair_fraction','pair_and_peak_fraction']);w.writerows(rows)

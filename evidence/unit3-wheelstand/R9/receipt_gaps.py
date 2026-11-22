"""Predict receiver blind intervals from a separate rest recording and one phase observation."""
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[1]/'unit3-cube-accelerometer'))
from packet_cadence import clocks


def gaps(times):
    ends=np.flatnonzero(np.diff(times)>.15)+1
    return times[ends-1],times[ends],ends


if __name__=='__main__':
    start,end,_=gaps(clocks('resting')[0])
    period=np.polyfit(np.arange(len(end)),end,1)[0]
    width=np.median(end-start)
    print('Training period and median gap duration:',period,width)
    fig,axes=plt.subplots(2,1,figsize=(9,6),layout='constrained')
    with (here/'receipt-gaps.csv').open('w') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(('run','scored_gaps','predicted_gaps','median_abs_end_error_ms','max_abs_end_error_ms',
                         'intersection_over_union','total_visible_span_s','observed_gap_seconds'))
        for name in ('R5','R6','R7','R8','R9'):
            directory=here.parent/name
            packets=np.genfromtxt(directory/'cube.csv.gz',delimiter=',',names=True)[::3]
            t=packets['uptime']-packets['uptime'][0]
            starts,ends,indices=gaps(t)
            # Condition only on the first returning packet. Score complete subsequent cycles.
            horizon=t[-1]
            predict=ends[0]+period*np.arange(1,1+int((horizon-ends[0])/period))
            actual=ends[1:]
            predicted_starts=predict-width
            overlap=np.maximum(0,np.minimum(actual[:,None],predict)-np.maximum(starts[1:,None],predicted_starts)).sum()
            union=(actual-starts[1:]).sum()+len(predict)*width-overlap
            nearest=predict[np.argmin(abs(actual[:,None]-predict),axis=1)]
            errors=(nearest-actual)*1000
            writer.writerow((name,len(actual),len(predict),np.median(abs(errors)),max(abs(errors)),
                             overlap/union,horizon,sum(ends-starts)))
            axes[0].plot(actual,errors,'.-',ms=3,label=name)
            if name=='R8':
                with (directory/'actions.csv').open() as af:
                    zero=next(float(r['sec']) for r in csv.DictReader(af) if r['action']=='roll cube')
                for i in indices:
                    left,right=packets['sec'][[i-1,i]]-zero
                    if left<5.3 and right>2.5:print('R8 host receipt gap:',left,right)
                for y,l,r in [(0,starts[1:],actual),(1,predicted_starts,predict)]:
                    axes[1].broken_barh(list(zip(l,r-l)),(y-.3,.6))
    axes[0].set(xlabel='Seconds from first packet',ylabel='Predicted return error (ms)')
    axes[0].legend(ncol=5)
    axes[1].set(xlabel='Seconds from first R8 packet',yticks=[0,1],yticklabels=['Observed gaps','Predicted gaps'])
    fig.savefig(here/'receipt-gaps.png',dpi=150)

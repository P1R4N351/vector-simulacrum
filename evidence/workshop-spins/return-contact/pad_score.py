from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
fig,ax=plt.subplots(1,2,figsize=(10,4),layout='constrained')
results=[]
for width in ('9','6','3','9-filtered'):
    with (here/f'pad-{width}.csv').open() as f:
        rows=list(csv.DictReader(f))
    for start in ('seated','direct','diagonal−','diagonal+'):
        r=[s for s in rows if s['start']==start]
        def vals(key):return np.array([float(s[key]) for s in r])
        t,p,on=vals('time_s'),vals('pitch_deg'),vals('contact')
        rest=np.array([s['phase']=='rest' for s in r])
        out=dict(width_mm=width,start=start,rest_on=int(on[rest].sum()),rest_packets=int(rest.sum()),
                 depth_at_flag_mm='',peak_deg='',peak_to_flag_mm='',peak_to_flag_s='')
        hits=np.flatnonzero(on)
        if start!='seated' and hits.size:
            i=hits[0]
            j=np.flatnonzero((p[:i]>-5)&(p[1:i+1]<=-5))[-1]+1
            k=j+np.argmin(p[j:i+1])
            speed=(vals('left_mm_s')+vals('right_mm_s'))/2
            dist=np.r_[0,-np.cumsum((speed[:-1]+speed[1:])/2*np.diff(t))]
            out.update(depth_at_flag_mm=vals('depth_mm')[i],peak_deg=p[k],peak_to_flag_mm=dist[i]-dist[k],peak_to_flag_s=t[i]-t[k])
            plot=ax[int(start!='direct')]
            plot.plot(dist[k:i+1]-dist[k],p[k:i+1],'.-',ms=3,label='±9 mm, filtered' if width=='9-filtered' else f'±{width} mm')
        results.append(out)
with (here/'pad-summary.csv').open('w') as f:
    writer=csv.DictWriter(f,fieldnames=results[0])
    writer.writeheader();writer.writerows(results)
for plot,title in zip(ax,['Direct return','Negative diagonal']):
    plot.set(xlim=(-1,38),xlabel='Wheel travel after peak pitch (mm)',ylabel='Pitch (degrees)',title=title)
    plot.axvspan(33.86,36.32,alpha=.14,color='green',label='Physical flag range')
    plot.legend(fontsize=8)
fig.savefig(here/'pad-window.png',dpi=165)

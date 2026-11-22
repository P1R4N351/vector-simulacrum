"""Compare SDK samples on their native frame and host receipt clocks."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
def read(path):
    with path.open() as handle:
        return list(csv.DictReader(handle))

native = read(here.parent/'hw-rec-20261119/unit2-native-stops.csv')
received = read(here/'sdk-stop-receipts.csv')

def measure(t, speed, gyro, threshold):
    i = np.flatnonzero((speed[:-1]>=threshold)&(speed[1:]<threshold))[-1]
    start = np.interp(threshold,speed[i:i+2][::-1],t[i:i+2][::-1])
    end = start+.08
    assert end<=t[-1]
    pts = np.r_[start,t[(t>start)&(t<end)],end]
    return np.degrees(np.trapezoid(np.interp(pts,t,gyro),pts)),start

fig, axes = plt.subplots(2,2,figsize=(8,5.8),layout='constrained')
with (here/'sdk-stop-clock.csv').open('w',newline='') as handle:
    writer=csv.writer(handle)
    writer.writerow(['spin','dense_deg','sdk_on_robot_clock_deg','sdk_on_host_clock_deg','sample_selection_error_deg',
                     'receipt_timing_error_deg','matched_points','unmatched_between_first_last'])
    for label,ax in zip(dict.fromkeys(r['turn'] for r in native),axes.flat):
        rows=[r for r in native if r['turn']==label]
        t=np.array([float(r['ms_from_last_edge'])/1000 for r in rows])
        values=np.array([[float(r[k]) for k in ('left_mmps','right_mmps','head_gx','head_gy','head_gz')] for r in rows])
        speed=np.abs(values[:,:2]).mean(axis=1)
        g=values[:,-1]/np.cos([float(r['head_rad']) for r in rows])
        g=(g-np.median(g[t>.25]))*(1 if label.startswith('ccw') else -1)
        limit=.8*speed[(t>=-.5)&(t<=-.25)].mean()
        host=[];indices=[];order=[]
        for pos,r in enumerate(r for r in received if r['spin']==label):
            key=[float(r[k]) for k in ('wl','wr','gx','gy','gz')]
            match=np.flatnonzero(np.all(np.abs(values-key)<1e-9,axis=1))
            if len(match)==1 and -.45<t[match[0]]<.16:
                indices.append(match[0]);host.append(float(r['host_s']));order.append(pos)
        ix=np.array(indices)
        assert np.all(np.diff(ix)>0)
        data=[(t,speed,g),(t[ix],speed[ix],g[ix]),(np.asarray(host),speed[ix],g[ix])]
        estimates=[]
        for (clock,w,y),legend in zip(data,('Native','SDK / robot clock','SDK / host clock')):
            angle,start=measure(clock,w,y,limit)
            estimates.append(angle)
            ax.plot(1000*(clock-start),np.degrees(y),'.-',label=legend)
        dense,sparse,receipt=estimates
        writer.writerow([label,*estimates,sparse-dense,receipt-sparse,len(ix),order[-1]-order[0]+1-len(ix)])
        ax.set(xlim=(-90,110),xlabel='ms from wheel crossing',ylabel='Yaw (°/s)',title=label)
        ax.axvspan(0,80,color='grey',alpha=.1)
axes[0,0].legend(fontsize=8)
fig.savefig(here/'sdk-stop-clock.png',dpi=160)

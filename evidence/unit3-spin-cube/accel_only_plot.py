"""Compare reconstructed taps with physical counter changes in packet order."""
import csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parent
def read(name):
    return np.genfromtxt(root/name,names=True,dtype=None,delimiter=',',encoding='utf8')
slots=read('cube.csv.gz')
actual=read('counter-uptime.csv')
replay=read('accel-only-taps.csv')
fig,axes=plt.subplots(2,3,figsize=(11,5.6),layout='constrained',sharex=True)
results=[]
for ax,event in zip(axes.flat,actual,strict=True):
    trial=event['trial']
    sample=slots[slots['trial']==trial]
    packets=sample['packet_s'][::3]
    assert np.array_equal(sample['sample'],np.tile(np.arange(3),len(packets)))
    j=np.flatnonzero(packets==event['packet_s']).item()
    # Keep packet times separate from the slot sequence; exact frame times are unknown.
    ix=np.arange(3*j-8,3*j+13)
    ax.plot(ix-3*j,sample['z'][ix]/8192,'.-',color='0.35')
    ax.axvspan(-.5,2.5,color='tab:orange',alpha=.22)
    ax.set_title(f"{trial} • tap counter {event['tap_count']}")
    for mode,marker in [('hold','s'),('linear','x')]:
        candidates=replay[(replay['mode']==mode)&(replay['trial']==trial)]
        matched=[r for r in candidates if r['packet_s'] in packets[max(0,j-1):j+2]]
        assert len(matched)<=1
        delta=''
        if matched:
            r=matched[0];k=np.flatnonzero(packets==r['packet_s']).item()*3+r['slot']
            ax.scatter(k-3*j,sample['z'][k]/8192,marker=marker,label=mode,zorder=4)
            delta=1000*(r['packet_s']-event['packet_s'])
        results.append([trial,event['tap_count'],mode,int(bool(matched)),delta])
    if ax.get_legend_handles_labels()[0]:ax.legend(fontsize=7)
for ax in axes[-1]:ax.set_xlabel('Sample index from first slot of tap packet')
for ax in axes[:,0]:ax.set_ylabel('Transmitted cube Z (g)')
fig.savefig(root/'accel-only.png',dpi=155)
with (root/'accel-only-scores.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n');w.writerow('trial physical_counter mode matched delta_receipt_ms'.split());w.writerows(results)
for mode in ['hold','linear']:
    hits=sum(r[3] for r in results if r[2]==mode)
    print(mode,hits,'matched;',np.count_nonzero(replay['mode']==mode)-hits,'extra')

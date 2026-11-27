"""Compare byte identity, voltage markers, and accelerometer gaps in physical radio logs."""
import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).parent
def packets(run):
    if run in ('resting','pickup','undock-check'):
        return [(float(t),bytes.fromhex(p)) for _,t,p in map(str.split,(root/(run+'.tsv')).read_text().splitlines())]
    with gzip.open(root.parent/'unit3-physical-repeats'/run/'radio.csv.gz','rt') as f:
        return [(float(r['robot_uptime_s']),bytes.fromhex(r['payload_hex'])) for r in csv.DictReader(f)]

fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
with (root/'voltage-restarts.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['run','long_gaps','voltage_adjacent_to_restart','duplicate_at_restart','both','neither','duplicates_elsewhere','voltage_intervals','unique_count_within_firmware_bounds'])
    for run in ('resting','pickup','undock-check','A','E','F'):
        rows=packets(run)
        acc=[i for i,(_,p) in enumerate(rows) if p[0]==2 and len(p)==20]
        volt={i for i,(_,p) in enumerate(rows) if p[0]==4 and len(p)==4}
        repeats={i for i in acc if i and rows[i-1][1]==rows[i][1]}
        starts=[(a,b) for a,b in zip(acc,acc[1:]) if rows[b][0]-rows[a][0]>.15]
        with_v=sum((b-1 in volt or b+1 in volt) for a,b in starts)
        with_d=sum(b+1 in repeats for a,b in starts)
        both=sum((b-1 in volt or b+1 in volt) and b+1 in repeats for a,b in starts)
        match=0
        for a,b in zip(sorted(volt),sorted(volt)[1:]):
            # A voltage marker may lie immediately before or after the first accel packet.
            cycles=round((rows[b][0]-rows[a][0])/1.22746)
            count=sum(a<i<b for i in acc)
            unique=count-sum(a<i<b for i in repeats)
            match+=int(100*cycles//3<=unique<=(100*cycles+2)//3)
            axes[0].plot(cycles,count,'x',c='darkorange')
            axes[0].plot(cycles,unique,'+',c='teal')
        fields=[run,len(starts),with_v,with_d,both,len(starts)-with_v-with_d+both,len(repeats)-with_d,len(volt)-1,match]
        w.writerow(fields)
        print(fields)
        axes[1].bar(run,with_v,label='Voltage beside restart' if run=='resting' else None,color='steelblue')
        axes[1].bar(run,with_d-both,bottom=with_v,label='Repeat after gap only' if run=='resting' else None,color='orange')
axes[0].plot([1,3],[100/3,100],':',c='black',label='Firmware mean')
axes[0].plot([],[],'x',c='darkorange',label='All receipts')
axes[0].plot([],[],'+',c='teal',label='Excluding adjacent exact repeats')
axes[0].set(xlabel='Approximate restart cycles between voltage markers',ylabel='Accel reports between markers')
axes[1].set(ylabel='Long gaps with marker',xlabel='Physical recording')
for ax in axes: ax.legend(fontsize=8)
fig.savefig(root/'voltage-restarts.png',dpi=155)

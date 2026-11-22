"""Forecast later accelerometer outage windows from the first observed restart."""
import csv
import gzip
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from packet_cadence import here

def load(name):
    if name in ('resting','pickup','undock-check'):
        return [(float(t),p) for _,t,p in (line.split() for line in (here/(name+'.tsv')).read_text().splitlines())]
    with gzip.open(here.parent/'unit3-physical-repeats'/name/'radio.csv.gz','rt') as f:
        return [(float(r['robot_uptime_s']),r['payload_hex']) for r in csv.DictReader(f)]

def windows(log):
    a=np.array([t for t,p in log if len(p)==40 and p.startswith('02')])
    gaps=np.flatnonzero(np.diff(a)>.15)
    return a[gaps],a[gaps+1]
beg,end=windows(load('resting'))
ordinal=np.arange(len(end))
period=np.polyfit(ordinal,end-end[0],1)[0]
duration=np.median(end-beg)
fig,axes=plt.subplots(2,1,figsize=(8,6),layout='constrained')
with (here/'receiver-phase.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow('run long_gaps battery_packets gaps_with_battery period_s median_duration_s forecast_restart_max_ms forecast_start_max_ms window_iou'.split())
    for name in ('undock-check','pickup','A','E','F'):
        log=load(name)
        start,restart=windows(log)
        battery=np.array([t for t,p in log if p.startswith('04') and len(p)==8])
        # Compare by record order because the 10 ms uptime clock can put two packets at the same time.
        between=0
        last=None
        found=False
        for t,p in log:
            if p.startswith('04') and len(p)==8:
                found=True
            if p.startswith('02') and len(p)==40:
                if last is not None and t-last>.15:
                    between+=int(found)
                last=t
                found=False
        times=restart[1:]-restart[0]
        pred=period*np.arange(1,len(restart))
        e=pred-times
        predicted_start=pred+restart[0]-duration
        predicted_end=pred+restart[0]
        common=np.maximum(0,np.minimum(restart[1:],predicted_end)-np.maximum(start[1:],predicted_start))
        union=duration+restart[1:]-start[1:]-common
        axes[0].plot(times,1000*e,'.-',label=name)
        axes[1].plot(times,common/union,'.-')
        w.writerow((name,len(restart),len(battery),between,period,duration,1000*max(abs(e)),
                    1000*max(abs(predicted_start-start[1:])),sum(common)/sum(union)))
axes[0].axhline(0,color='.6',lw=.7)
axes[0].set_ylabel('Restart prediction error (ms)')
axes[1].set(xlabel='Seconds since first observed restart',ylabel='Gap window overlap / union',ylim=(0,1.04))
axes[0].legend(ncol=5)
fig.savefig(here/'receiver-phase.png',dpi=160)

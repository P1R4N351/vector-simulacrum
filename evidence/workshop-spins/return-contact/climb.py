"""Partition reverse travel at pitch landmarks and check a separate camera recording."""
import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
camera_dir = here.parents[1] / 'unit3-w34-return'
fig, axes = plt.subplots(1, 2, figsize=(11, 4.6), layout='constrained')
scores = []
for name, source in [
        ('Workshop 1', here.parent/'states.csv'),
        ('Workshop 2', here.parent/'speed-holdout/states.csv'),
        ('Workshop 3', here.parent/'interleaved/states.csv'),
        ('Driven, direct return', here/'autonomous-driven.csv'),
        ('Driven, diagonal return', here/'autonomous-driven-60.csv'),
        ('Shelf W34', camera_dir/'states.csv.gz')]:
    with (gzip.open(source, 'rt') if source.suffix=='.gz' else source.open()) as f:
        rows = [r for r in csv.DictReader(f) if r.get('phase','return')=='return']
    def col(key):
        return np.array([float(r[key]) for r in rows])
    shelf = name=='Shelf W34'
    t = col('receipt_s' if shelf else 'time_s')
    w = (col('left_mmps' if shelf else 'left_mm_s')+col('right_mmps' if shelf else 'right_mm_s'))/2
    pitch = np.rad2deg(col('pitch_rad'))
    on = col('on')!=0 if 'on' in rows[0] else (col('status' if shelf else 'flags').astype(int)&4096)!=0
    i = np.flatnonzero(on)[0]
    t = t-t[i]
    j = np.flatnonzero((pitch[1:i+1]<=-5)&(pitch[:i]>-5))[-1]+1
    start = np.interp(-5, pitch[[j,j-1]], t[[j,j-1]])
    length = np.r_[0,-np.cumsum(np.diff(t)*(w[1:]+w[:-1])/2)]
    travel = length-np.interp(start,t,length)
    peak = j+np.argmin(pitch[j:i+1])
    before, after = travel[peak], travel[i]-travel[peak]
    smooth = np.convolve(pitch,np.ones(5)/5,mode='same')
    smooth_peak = j+np.argmin(smooth[j:i-1])
    if 'gyro_y' in rows[0]:
        head=col('head_rad')
        gy=col('gyro_y')
        print(name,'peak to contact: head delta, gyro Y integral (degrees)', np.degrees(head[i]-head[peak]),np.degrees(np.sum((gy[peak:i]+gy[peak+1:i+1])*.5*np.diff(t[peak:i+1]))))
    scores.append(dict(smooth_after_peak_mm=travel[i]-travel[smooth_peak],series=name,to_peak_mm=before,after_peak_mm=after,total_mm=travel[i],
                       peak_deg=pitch[peak],at_flag_deg=pitch[i],peak_before_flag_s=-t[peak]))
    ok=(t>=start)&(t<=0)
    axes[0].plot(travel[ok],pitch[ok],label=name)
    axes[0].scatter(travel[peak],pitch[peak],s=15)
    if shelf:
        cam=np.genfromtxt(camera_dir/'hubs.csv',names=True,delimiter=',')
        scale=48/np.median(cam['rear_x']-cam['front_x'])
        mid=(cam['rear_x']+cam['front_x'])/2
        for a,b in zip(range(len(mid)-1),range(1,len(mid))):
            if cam['t_s'][b] <= 0:
                camera_delta = (mid[b]-mid[a])*scale
                encoder_delta = np.diff(np.interp(cam['t_s'][[a,b]],t,travel))[0]
                print("Measured frame pair (seconds), optical and encoder mm:",cam['t_s'][[a,b]],camera_delta,encoder_delta)
        def optical(tm):
            if not cam['t_s'][0]<=tm<=cam['t_s'][-1]: raise ValueError('Outside camera coverage')
            return np.interp(tm,cam['t_s'],mid)*scale
        print('Camera travel estimated at pitch landmarks (mm):',optical(t[peak])-optical(start),optical(0)-optical(t[peak]))
for s in scores:
    print(s)
with (here/'climb.csv').open('w') as f:
    wr=csv.DictWriter(f,fieldnames=scores[0]);wr.writeheader();wr.writerows(scores)
x=np.arange(len(scores))
before=[s['to_peak_mm'] for s in scores]
axes[1].bar(x,before,label='−5° to peak')
axes[1].bar(x,[s['after_peak_mm'] for s in scores],bottom=before,label='Peak to charger flag')
axes[1].set_xticks(x,['W1','W2','W3','Sim\ndirect','Sim\ndiagonal','Shelf\nW34'])
axes[1].set_ylabel('Integrated reverse wheel travel (mm)')
axes[0].set(xlabel='Encoder travel since −5° crossing (mm)',ylabel='Reported pitch (degrees)')
for ax in axes:ax.legend(fontsize=8)
fig.savefig(here/'climb.png',dpi=170)

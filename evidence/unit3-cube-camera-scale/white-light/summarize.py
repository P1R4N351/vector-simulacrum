"""Compare LED blocks after a 1 s settling interval, using exact image stamps."""
import argparse
import csv
from pathlib import Path
import cv2
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
parser=argparse.ArgumentParser()
parser.add_argument('--images',type=Path)
args=parser.parse_args()
events=list(csv.DictReader((here/'events.csv').open()))
starts=[float(r['seconds']) for r in events if r['kind']=='block']
visible={r['value'] for r in events if r['kind']=='observation'}
if args.images:
    with (here/'pixels.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['stamp','top_strip','front_marker'])
        for r in events:
            if r['kind']!='frame':continue
            im=cv2.imread(str(args.images/(r['value']+'.jpg')),0)
            w.writerow([r['value'],np.mean(im[199:218,375:455]),np.mean(im[225:281,389:445])])
pixels={r['stamp']:r for r in csv.DictReader((here/'pixels.csv').open())}
fig,ax=plt.subplots(2,1,figsize=(8,5),layout='constrained')
for i,t in enumerate(starts):
    group=[r for r in events if 1<=float(r['seconds'])-t<2.95]
    images=[r['value'] for r in group if r['kind']=='frame']
    accel=np.vstack([np.frombuffer(bytes.fromhex(r['value'])[2:],'<i2').reshape(3,3)/8192 for r in group if r['kind']=='accel'])
    hits=sum(stamp in visible for stamp in images)
    means=[np.mean([float(pixels[s][k]) for s in images]) for k in ('top_strip','front_marker')]
    print(i,len(images),hits,np.round(means,1),len(accel),np.round(accel.mean(axis=0)*1000,1))
    for j,v in enumerate(means):ax[0].plot(i,v,['C0o','C1s'][j],label=['Top strip','Front glyph'][j] if i==0 else None)
    ax[1].bar(i,hits/len(images),color='gray' if i%2==0 else '#29799a')
ax[0].legend();ax[0].set_ylabel('Mean JPEG gray value')
ax[1].set(ylabel='Reported cube / frames',ylim=(0,1.08),xticks=range(7),xticklabels=['Off','White','Off','White','Off','White','Off'])
fig.savefig(here/'light-response.png',dpi=160)

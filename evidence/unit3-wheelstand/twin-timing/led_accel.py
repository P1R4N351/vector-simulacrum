"""Compare each green dwell with its two neighboring dark dwells."""
import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent


def main():
    stem = 'led-accel-repeat' if '--repeat' in sys.argv else 'led-accel'
    rows=list(csv.DictReader((here/(stem+'.csv')).open()))
    lights=[(float(r['sec']),int(r['value'])) for r in rows if r['kind']=='light']
    sensor=[]
    volts=[]
    for r in rows:
        if r['kind']!='packet':
            continue
        b=bytes.fromhex(r['value'])
        t=float(r['sec'])
        if len(b)==20 and b[0]==2:
            sensor.append((t,b[1],np.frombuffer(b[2:],'<i2').reshape(3,3)/8192))
        elif len(b)==4 and b[0]==4:
            volts.append((t,int.from_bytes(b[2:],'little')*3.6/1024))
    assert [s for t,s in lights]==[0,1,0,1,0,1,0,1,0]
    means=[]
    for t,_ in lights:
        a=np.vstack([v for stamp,_,v in sensor if .15<=stamp-t<.70])
        means.append(a.mean(axis=0))
    means=np.array(means)
    contrasts=(means[1::2]-(means[:-1:2]+means[2::2])/2)*1000
    t,taps,v=zip(*sensor)
    print('packets',len(sensor),'tap changes',np.count_nonzero(np.diff(taps)),'rail readings',volts)
    print('paired mg XYZ',np.round(contrasts,2))
    fig,axes=plt.subplots(2,1,figsize=(8,5),layout='constrained')
    for i,l in enumerate('XYZ'):
        packet_means=np.mean(v,axis=1)[:,i]
        axes[0].plot(t,1000*(packet_means-np.median(packet_means)),'.',ms=2,label=l)
        axes[1].plot(range(1,5),contrasts[:,i],'o-',label=l)
    for idx in range(1,8,2):
        axes[0].axvspan(lights[idx][0],lights[idx+1][0],color='green',alpha=.13)
    axes[0].set(xlabel='Host time (s)',ylabel='Centered packet mean (mg)')
    axes[1].set(xlabel='Green dwell',ylabel='Green minus adjacent dark (mg)',xticks=range(1,5))
    axes[1].axhline(0,color='.5',lw=.7)
    axes[0].legend(ncol=3)
    fig.savefig(here/(stem+'.png'),dpi=160)

if __name__=='__main__':
    main()

"""Bracket autonomous LED transitions inside direct BLE accelerometer gaps."""
from pathlib import Path
import csv
import subprocess
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
from shelf_view import ShelfView,SERIALS

here=Path(__file__).resolve().parent


def optical(path,start):
    t=np.fromstring(subprocess.check_output(['ffprobe','-v','error','-show_entries','packet=pts_time','-of','csv=p=0',str(path)],text=True),sep='\n')
    cap=cv2.VideoCapture(str(path))
    levels=[]
    # Locate the modulated patch near the known cube, using only the first two seconds.
    x,y=(301,156) if path.stem==SERIALS[0] else (222,89)
    for _ in t:
        ok,frame=cap.read()
        assert ok
        if not levels:
            view=ShelfView(path.stem,frame)
        scaled=cv2.resize(view.apply(frame),(500,220))
        b,g,r=cv2.split(scaled[y-24:y+25,x-24:x+25].astype(float))
        levels.append(g-(r+b)/2)
    cap.release()
    levels=np.array(levels)
    training=levels[(t>start+.2)&(t<start+2)]
    amplitude=np.percentile(training,90,axis=0)-np.percentile(training,10,axis=0)
    yy,xx=np.unravel_index(np.argmax(cv2.blur(amplitude,(5,5))[2:-2,2:-2]),(45,45))
    return t,levels[:,yy:yy+5,xx:xx+5].mean(axis=(1,2))


def main(folder):
    rows=list(csv.DictReader((folder/'receipts.csv').open()))
    start=next(float(r['sec']) for r in rows if r['kind']=='start')
    end=next(float(r['sec']) for r in rows if r['kind']=='stop')-start
    packets=np.array([float(r['sec'])-start for r in rows if r['kind']=='packet' and r['value'].startswith('02')])
    gap=np.where((np.diff(packets)>.15)&(packets[:-1]>0)&(packets[1:]<end))[0]
    fig,ax=plt.subplots(figsize=(10,3.5),layout='constrained')
    bands=[]
    coverage=[]
    with (here/'blink-clock-traces.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['camera','relative_s','green_contrast'])
        for side,serial in zip('AB',SERIALS):
            t,signal=optical(folder/(serial+'.nut'),start);t-=start
            w.writerows((side,s,v) for s,v in zip(t,signal))
            active=(t>.25)&(t<end-.25)
            low,high=np.percentile(signal[active],[10,90])
            assert high-low>10
            norm=(signal-low)/(high-low)
            state=norm>.5
            edges=np.flatnonzero((state[1:]!=state[:-1]) & active[1:])
            coverage.append(t)
            bands.append([(t[i],t[i+1],int(state[i+1])) for i in edges])
            ax.plot(t,norm,'.-',lw=.7,ms=2,label=side)
    scores=[]
    for i in gap:
        lo,hi=packets[i:i+2]
        ax.axvspan(lo,hi,color='gray',alpha=.2)
        # Both whole frame brackets must lie within the same packet gap and overlap.
        pairs=[(a,b,c,d) for a,b,k in bands[0] for c,d,j in bands[1]
               if k==j and max(a,c)<=min(b,d) and lo<min(a,c) and max(b,d)<hi]
        lengths=[]
        ns=[]
        for t,edges in zip(coverage,bands):
            ns.append(int(np.sum((t>=lo)&(t<=hi))))
            mid=np.array([(a+b)/2 for a,b,_ in edges])
            j=int(np.searchsorted(mid,(lo+hi)/2))
            lengths.append(round(1000*(mid[j]-mid[j-1]),1) if 0<j<len(mid) else None)
        scores.append((lo,hi,*ns,len(pairs) if min(ns)>0 else None,*lengths))
    print('gaps and confirmed edges',scores)
    ax.set(xlim=(0,end),xlabel='Seconds from animation start acknowledgement',ylabel='Normalized green contrast')
    ax.legend()
    fig.savefig(here/'blink-clock.png',dpi=160)
    with (here/'blink-clock-gaps.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['start_s','end_s','frames_a','frames_b','both_view_edges','dwell_a_ms','dwell_b_ms']);w.writerows(scores)


if __name__=='__main__':
    main(Path(sys.argv[1]))

"""Extract green LED modulation from two camera clips with preserved timestamps."""
import csv
from pathlib import Path
import subprocess
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np


def extract(path, commands):
    times = np.fromstring(subprocess.check_output(['ffprobe','-v','error','-show_entries',
        'packet=pts_time','-of','csv=p=0',str(path)], text=True), sep='\n')
    cap = cv2.VideoCapture(str(path))
    images = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        b,g,r = cv2.split(cv2.resize(frame,(400,225)).astype(np.float32))
        images.append(g-(r+b)/2)
    cap.release()
    values = np.stack(images)
    assert len(times)==len(values)
    def plateau(start,end):
        return np.median(values[(times>start)&(times<end)],axis=0)
    base = plateau(commands[0]-.55,commands[0]-.15)
    on = plateau(commands[0]+.15,commands[1]-.05)
    y,x = np.unravel_index(np.argmax(cv2.blur(on-base,(3,3))),on.shape)
    signal = values[:,max(0,y-2):y+3,max(0,x-2):x+3].mean(axis=(1,2))
    baseline = np.median(signal[(times>commands[0]-.55)&(times<commands[0]-.15)])
    bright = np.median(signal[(times>commands[0]+.15)&(times<commands[1]-.05)])
    norm = (signal-baseline)/(bright-baseline)
    return times,norm,(x,y),bright-baseline


def main(folder,output):
    with (folder/'log').open() as f:
        commands = [float(line.split()[1]) for line in f if line.startswith(('0 ','1 '))][1:]
    assert len(commands)==6
    fig,ax = plt.subplots(figsize=(8,3.6),layout='constrained')
    traces = []
    for side,serial in zip('AB',['222AD609','3815D409']):
        t,s,xy,contrast = extract(folder/(serial+'.nut'),commands)
        print(side,xy,'contrast',contrast)
        ax.plot(t-commands[0],s,'.-',ms=3,label=side)
        edges=[]
        for i,event in enumerate(commands):
            mask=(t[1:]>event-.25)&(t[1:]<event+.3)
            crossing = ((s[:-1]<.5)&(s[1:]>=.5) if i%2==0 else (s[:-1]>=.5)&(s[1:]<.5))
            matches=np.flatnonzero(mask&crossing)
            assert len(matches)==1,(side,event,matches)
            k=matches[0]
            edges.append((t[k]-commands[0],t[k+1]-commands[0]))
        traces.append(edges)
    with (output/'cube-led-edges.csv').open('w') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(['transition','a_before_s','a_after_s','b_before_s','b_after_s','b_minus_a_midpoint_ms'])
        for i,(a,b) in enumerate(zip(*traces)):
            delta=1000*(np.mean(b)-np.mean(a))
            writer.writerow([i+1,*a,*b,delta])
            print(i+1,round(delta,1),'intervals overlap',max(a[0],b[0])<=min(a[1],b[1]))
    ax.set(xlabel='Seconds from first LED-on write acknowledgement',ylabel='Normalized green contrast',xlim=(-.45,commands[-1]-commands[0]+.55))
    ax.legend(title='Camera')
    fig.savefig(output/'cube-led-pulses.png',dpi=160)


if __name__=='__main__':
    main(Path(sys.argv[1]),Path(sys.argv[2]))

"""Create a 20 fps two-sided review clip from native camera packet timestamps."""
import csv
import gzip
from pathlib import Path
import subprocess
import sys
import cv2
import numpy as np
here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent/'unit3-wheelstand/twin-timing'))
from shelf_view import ShelfView,SERIALS
folder=Path(sys.argv[1])
import json
zero=next(r['t'] for l in (folder/'run.jsonl').read_text().splitlines() if l.startswith('{') and (r:=json.loads(l)).get('action')=='lift 0.25')
clock=np.genfromtxt(folder/'clock.csv',delimiter=',',names=True)
state=np.genfromtxt(here/'encoder.csv',delimiter=',',names=True)
times=np.arange(-.5,2.45,.05)
streams=[]
for name,serial in zip('AB',SERIALS):
    path=folder/(name+'.nut')
    pts=np.fromstring(subprocess.check_output(['ffprobe','-v','error','-show_entries','packet=pts_time','-of','csv=p=0',str(path)],text=True),sep='\n')
    pts+=np.median(clock['unix']-clock['monotonic'])-zero
    streams.append([cv2.VideoCapture(str(path)),[int(np.argmin(abs(pts-t))) for t in times],-1,None,serial])
enc=subprocess.Popen(['ffmpeg','-loglevel','error','-y','-f','rawvideo','-pix_fmt','bgr24','-s','960x400','-r','20','-i','-','-c:v','libx264','-pix_fmt','yuv420p','-crf','22','-movflags','+faststart',str(here/'docked-contact.mp4')],stdin=subprocess.PIPE)
for n,t in enumerate(times):
    panels=[]
    for i,s in enumerate(streams):
        cap,indices,j,view,serial=s
        while j<indices[n]:
            assert cap.grab()
            j+=1
        ok,im=cap.retrieve()
        assert ok
        if view is None:view=ShelfView(serial,im)
        s[2:4]=[j,view]
        x=25 if i==0 else 520
        im=view.apply(im)[:400,x:x+480].copy()
        cv2.putText(im,f'{"AB"[i]}   {t:.1f} s   {np.interp(t,state["seconds"],state["lift_mm"]):.1f} mm',(8,387),0,.65,(0,230,255),1,cv2.LINE_AA)
        panels.append(im)
    enc.stdin.write(np.hstack(panels).tobytes())
enc.stdin.close()
assert enc.wait()==0
for s in streams:s[0].release()

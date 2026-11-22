"""Summarize two real lift sweeps and choose synchronized settled camera frames."""
import csv
import gzip
import json
from pathlib import Path
import subprocess
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent/'unit3-wheelstand/twin-timing'))
from shelf_view import ShelfView, SERIALS
folder=Path(sys.argv[1])
raw=[json.loads(line) for line in (folder/'sweep.jsonl').read_text().splitlines() if line.startswith('{')]
commands=[r for r in raw if r['kind']=='command']
arrival=[r for r in raw if r['kind']=='arrived']
assert len(commands)==len(arrival)==17 and all(r['result']==0 for r in arrival)
zero=commands[0]['t']
states=[r for r in raw if r['kind']=='state']
ts=np.array([r['t'] for r in states])
heights=np.array([r['lift'] for r in states])
with gzip.open(here/'state.csv.gz','wt') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['seconds','lift_mm','head_rad','pitch_rad','status','left_mmps','right_mmps'])
    w.writerows((r['t']-zero,*[r[k] for k in ('lift','head','pitch','status','left','right')]) for r in states)
clock=np.genfromtxt(folder/'clock.csv',names=True,delimiter=',')
delta=np.median(clock['unix']-clock['monotonic'])
target=np.array([r['t']+.35 for r in arrival])
poses=[]
for i,(c,a) in enumerate(zip(commands,arrival)):
    end=commands[i+1]['t'] if i+1<len(commands) else ts[-1]
    hold=(ts>=a['t']+.12)&(ts<=min(end,a['t']+.60))
    assert hold.any()
    poses.append([i,c['pos'],c['t']-zero,a['t']-zero,np.median(heights[hold]),int(hold.sum())])
frames={}
for name,serial in zip('AB',SERIALS):
    path=folder/(name+'.nut')
    t=np.fromstring(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_entries','packet=pts_time','-of','csv=p=0',str(path)],text=True),sep='\n')+delta
    assert np.all(np.diff(t)>0)
    ids=[np.argmin(abs(t-v)) for v in target]
    cap=cv2.VideoCapture(str(path))
    wanted=set(ids)
    for index in range(max(ids)+1):
        assert cap.grab()
        if index not in wanted:continue
        ok,im=cap.retrieve()
        assert ok
        if index==ids[0]:view=ShelfView(serial,im)
        frame=view.apply(im)
        frames[name,ids.index(index)]=frame
    cap.release()
    for r,ind,v in zip(poses,ids,target):
        r.extend([t[ind]-zero,1000*(t[ind]-v)])
with (here/'holds.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['hold','command_fraction','command_s','success_s','lift_mm','samples','A_s','A_delta_ms','B_s','B_delta_ms'])
    w.writerows(poses)
fig,ax=plt.subplots(figsize=(10,3.7),layout='constrained')
ax.plot(ts-zero,heights,lw=1.4,label='Encoder')
ax.scatter([r[6] for r in poses],[r[4] for r in poses],s=16,color='#b74719',label='Camera A hold samples')
ax.set(xlabel='Time since first command (s)',ylabel='Reported lift height (mm)',title='P2Y8 · two full docked sweeps with quarter-height holds');ax.legend()
fig.savefig(here/'lift-trace.png',dpi=160)
for iteration,indices in enumerate((range(9),range(8,17)),1):
    rows=[]
    for name,roi in [('A',(70,395,35,397)),('B',(595,965,5,355))]:
        x,b,y,d=roi
        panels=[]
        for idx in indices:
            im=frames[name,idx][y:d,x:b]
            im=cv2.resize(im,(200,205))
            cv2.putText(im,f'{poses[idx][4]:.1f} mm',(5,194),cv2.FONT_HERSHEY_SIMPLEX,.52,(0,225,255),1,cv2.LINE_AA)
            panels.append(im)
        rows.append(np.hstack(panels))
    cv2.imwrite(str(here/f'sweep-{iteration}.jpg'),np.vstack(rows),[cv2.IMWRITE_JPEG_QUALITY,92])
print('states',len(states),'wheel max',max(max(abs(r['left']),abs(r['right'])) for r in states),'on dock',sum(bool(r['status']&4096) for r in states))
print('held',[(r[0],round(r[4],2)) for r in poses])

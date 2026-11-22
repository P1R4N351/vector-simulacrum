"""Plot physical cube movement on BLE receipt time and both camera clocks."""
import csv
from pathlib import Path
import subprocess
import sys
import cv2
import matplotlib.pyplot as plt
import numpy as np
from shelf_view import ShelfView,SERIALS

def main(folder):
    with (folder/'receipts.csv').open() as f:
        rows=list(csv.DictReader(f))
    packets=[(float(r['sec']),bytes.fromhex(r['value'])) for r in rows if r['kind']=='packet' and r['value'].startswith('02')]
    t=np.array([s for s,b in packets])
    zero=t[0]
    t-=zero
    values=np.array([np.frombuffer(b[2:],'<i2').reshape(3,3) for s,b in packets],dtype=float)/8192
    base=np.median(values[t<1].reshape(-1,3),axis=0)
    angle=np.degrees(np.arccos(np.clip((values@base)/np.linalg.norm(base)/np.linalg.norm(values,axis=2),-1,1)))
    residual=np.linalg.norm(values-base,axis=2).max(axis=1)
    counter=np.array([b[1] for _,b in packets])
    print('packets, duration, base(g), max(g), max departure(g), angle, tap changes:',len(t),t[-1],base,np.linalg.norm(values,axis=2).max(),residual.max(),angle.max(),t[1:][np.diff(counter)!=0])
    print('departure >.15g grouped by 2s quiet:',[np.round([g[0],g[-1]],2) for g in np.split(t[residual>.15],np.where(np.diff(t[residual>.15])>2)[0]+1) if len(g)])
    fig,ax=plt.subplots(3,1,figsize=(10,6),sharex=True,layout='constrained')
    for j,k in enumerate('XYZ'):
        ax[0].plot(t,values[:,:,j].mean(axis=1),lw=.6,label=k)
    ax[0].legend(ncol=3)
    ax[0].set_ylabel('Packet mean (g)')
    ax[1].plot(t,np.linalg.norm(values,axis=2).max(axis=1),lw=.8)
    ax[1].set_ylabel('Packet peak (g)')
    for serial,(x,y) in zip(SERIALS,[(585,307),(425,218)]):
        path=folder/(serial+'.nut')
        times=np.fromstring(subprocess.check_output(['ffprobe','-v','error','-show_entries','packet=pts_time','-of','csv=p=0',str(path)],text=True),sep='\n')
        cap=cv2.VideoCapture(str(path))
        last=None
        motion=[]
        for i,stamp in enumerate(times):
            ok,frame=cap.read()
            if not ok: break
            if i%3:continue
            if last is None:view=ShelfView(serial,frame)
            crop=cv2.cvtColor(view.apply(frame)[y-90:y+90,x-110:x+110],cv2.COLOR_BGR2GRAY)
            if last is not None:
                motion.append((stamp-zero,cv2.absdiff(crop,last).mean()))
            last=crop
        cap.release()
        motion=np.asarray(motion)
        print(serial,'decoded samples',len(motion),'span',motion[[0,-1],0], 'maximum change',motion[:,1].max(),'at',motion[np.argmax(motion[:,1]),0])
        ax[2].plot(*motion.T,lw=.7,label=serial)
    ax[2].set(ylabel='Image change\n(gray levels)',xlabel='Seconds from first cube notification')
    ax[2].legend()
    fig.savefig(folder/'measured.png',dpi=150)
if __name__=='__main__':
    main(Path(sys.argv[1]))

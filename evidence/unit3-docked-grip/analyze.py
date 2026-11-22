"""Export the short docked lift attempt from recorder-time robot and cube events."""
import csv,json,struct,sys,subprocess
from pathlib import Path
import numpy as np
import cv2
import matplotlib.pyplot as plt
here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent/'unit3-wheelstand/twin-timing'))
from shelf_view import ShelfView,SERIALS
folder=Path(sys.argv[1])
events=[json.loads(s) for s in (folder/'run.jsonl').read_text().splitlines() if s.startswith('{')]
start,end=[r['t'] for r in events if r.get('action')=='lift 0.25']
states=np.array([[r['t']-start,r['data']['lift_height_mm'],r['data'].get('left_wheel_speed_mmps',0),r['data'].get('right_wheel_speed_mmps',0)] for r in events if r['kind']=='robot_state' and start-1<=r['t']<=end+1])
np.savetxt(here/'encoder.csv',states,delimiter=',',header='seconds,lift_mm,left_mmps,right_mmps',comments='')
rows=[]
for line in (folder/'packets.tsv').read_text().splitlines():
    ns,_,hexdata=line.split()
    packet=bytes.fromhex(hexdata)
    if len(packet)==20 and packet[0]==2:
        for j,axis in enumerate(np.array(struct.unpack('<9h',packet[2:])).reshape(3,3)):
            rows.append([int(ns)*1e-9-start,j,*axis])
raw=np.array(rows)
np.savetxt(here/'cube.csv',raw,delimiter=',',header='packet_receipt_s,index_in_packet,x_counts,y_counts,z_counts',comments='',fmt=['%.6f','%d','%d','%d','%d'])
axis=raw[:,2:]/8192
baseline=np.median(axis[raw[:,0]<0],axis=0)
delta=np.linalg.norm(axis-baseline,axis=1)
fig,axs=plt.subplots(2,1,sharex=True,figsize=(8,4.5),layout='constrained')
axs[0].plot(states[:,0],states[:,1],'.-')
axs[0].axhline(47,ls=':',color='gray',label='25% SDK target');axs[0].legend()
axs[0].set_ylabel('Lift encoder (mm)')
axs[1].scatter(raw[:,0],delta,s=5)
axs[1].set(xlabel='Seconds from lift command',ylabel='Cube acceleration\nchange from rest (g)')
for ax in axs: ax.axvspan(0,end-start,color='gray',alpha=.13)
fig.savefig(here/'sensors.png',dpi=160)
clock=np.genfromtxt(folder/'clock.csv',names=True,delimiter=',')
offset=np.median(clock['unix']-clock['monotonic'])-start
times=[-.15,.45,.95,1.50,2.15]
fig,axs=plt.subplots(2,len(times),figsize=(14,5),layout='constrained')
for i,(name,serial) in enumerate(zip('AB',SERIALS)):
    path=folder/(name+'.nut')
    pts=np.fromstring(subprocess.check_output(['ffprobe','-v','error','-show_entries','packet=pts_time','-of','csv=p=0',str(path)],text=True),sep='\n')+offset
    cap=cv2.VideoCapture(str(path)); view=None; prev=-1
    for ax,t in zip(axs[i],times):
        k=np.argmin(abs(pts-t))
        assert abs(pts[k]-t)<.1
        for _ in range(k-prev): assert cap.grab()
        ok,im=cap.retrieve();assert ok
        prev=k
        if view is None:view=ShelfView(serial,im)
        x=25 if i==0 else 490
        crop=view.apply(im)[65:335,x:x+520]
        ax.imshow(cv2.cvtColor(crop,cv2.COLOR_BGR2RGB));ax.axis('off')
        ax.set_title(f'{name}  {pts[k]:+.2f} s')
    cap.release()
fig.savefig(here/'contact.jpg',dpi=145)
print('result',end-start,'encoder pre/peak/post',np.median(states[states[:,0]<0,1]),max(states[:,1]),np.median(states[states[:,0]>end-start,1]),'packets',len(raw)//3,'cube deviation pre/raise max',max(delta[raw[:,0]<0]),max(delta[(raw[:,0]>=0)&(raw[:,0]<=end-start)]),'wheel max',abs(states[:,2:]).max())

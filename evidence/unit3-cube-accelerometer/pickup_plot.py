"""Line up cube packet receipt and SDK robot state on the recorder's clock."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from decode import decode

here=Path(__file__).resolve().parent
events=list(csv.DictReader((here/'action-times.csv').open()))
start=float(next(e['pod_unix_s'] for e in events if e['action']=='pickup'))
packets,_=decode(here/'pickup.tsv')
# Pod receipt and robot uptime are independent of the camera and cube's sample clock.
# Three slots arrive together. Retain batches instead of inventing exact slot timestamps.
t=np.repeat([p[0]*1e-9-start for p in packets],3)
g=np.vstack([np.frombuffer(p[3],dtype='<i2').reshape(3,3) for p in packets])/8192
robot=np.genfromtxt(here/'robot.csv',delimiter=',',names=True)
fig,(ax,bx)=plt.subplots(2,1,figsize=(10,6),sharex=True)
ax.plot(t,g,lw=.65,label=['Cube X','Cube Y','Cube Z'])
ax.plot(t,np.linalg.norm(g,axis=1),color='#393939',lw=.7,label='Magnitude')
ax.set_ylabel('Specific force (g)')
ax.legend(ncol=4)
bx.plot(robot['pod_unix_s']-start,robot['lift_mm'],color='#6a4193')
bx.set_ylabel('Reported lift height (mm)')
bx.set_xlabel('Seconds from pickup command')
for e in events:
 if e['event']!='begin' or e['action'] not in ('turn loaded','place'):continue
 x=float(e['pod_unix_s'])-start
 for axis in (ax,bx):axis.axvline(x,ls=':',color='.5')
 ax.text(x,1.02,'Turn' if e['action']=='turn loaded' else 'Place',transform=ax.get_xaxis_transform(),fontsize=8)
bx.set_xlim(-1,14)
fig.suptitle('Cube accelerometer during pickup, opposed 20° turns and placement',y=1)
fig.tight_layout()
fig.savefig(here/'pickup-overlay.png',dpi=160,bbox_inches='tight')
for lo,hi in [(-1,0),(6.5,8.3),(8.48,10.16),(10.17,13.04),(13.1,14)]:
 v=g[(t>=lo)&(t<hi)]
 if len(v): print(lo,hi,len(v),'median',np.round(np.median(v,axis=0),3),'peak',np.linalg.norm(v,axis=1).max())
i=np.argmax(np.linalg.norm(g,axis=1));print('peak',t[i],g[i])

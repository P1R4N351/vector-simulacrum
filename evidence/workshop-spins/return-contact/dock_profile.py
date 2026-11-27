"""Vertical sections through the dock asset and the canonical MuJoCo contact boxes."""
import csv
from hashlib import sha256
from itertools import product
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
assets = here.parents[2]/'web/public/robot'
obj = (assets/'sdk/Vic_charger_hp_geo.obj').read_bytes()
assert sha256(obj).hexdigest() == '2ccfb20d5ed4b793862f8deea940632a2f4f34a0dfc76f8fd6083e53419b3bb0'
vertices, faces = [], []
for line in obj.decode().splitlines():
    tok = line.split()
    if not tok:
        continue
    if tok[0]=='v':
        u,v,w = map(float,tok[1:4])
        vertices.append([w+81.2401,u,v+.5586])
    elif tok[0]=='f':
        ids = [int(t.split('/')[0])-1 for t in tok[1:]]
        faces.extend((ids[0],a,b) for a,b in zip(ids[1:],ids[2:]))
mesh = np.asarray(vertices)[np.asarray(faces)]
boxes=[]
dock=ET.parse(assets/'vector-free-dock.xml').find(".//body[@name='dock']")
for geom in dock.findall('geom'):
    if geom.get('contype')=='0' and geom.get('conaffinity')=='0':
        continue
    w,x,y,z=map(float,geom.get('quat','1 0 0 0').split())
    rot=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    size=np.fromstring(geom.get('size'),sep=' ')*1000
    pos=np.fromstring(geom.get('pos'),sep=' ')*1000
    for axis,sign in product(range(3),(-1,1)):
        pts=np.zeros((4,3));pts[:,axis]=sign
        pts[:,[(axis+1)%3,(axis+2)%3]]=[[-1,-1],[1,-1],[1,1],[-1,1]]
        pts=(pts*size)@rot.T+pos
        boxes.extend(pts[[[0,1,2],[0,2,3]]])
boxes=np.asarray(boxes)

def section(triangles, lateral):
    a,b,c=triangles.transpose(1,0,2)
    u,v=b-a,c-a
    det=u[:,0]*v[:,1]-u[:,1]*v[:,0]
    valid=(np.abs(det)>1e-10)&(triangles[:,:,1].min(axis=1)<=lateral)&(triangles[:,:,1].max(axis=1)>=lateral)
    a,u,v,det=a[valid],u[valid],v[valid],det[valid]
    heights=[]
    for x in stations:
        dx,dy=x-a[:,0],lateral-a[:,1]
        s=(dx*v[:,1]-dy*v[:,0])/det
        t=(dy*u[:,0]-dx*u[:,1])/det
        inside=(s>=-1e-8)&(t>=-1e-8)&(s+t<=1+1e-8)
        heights.append(max(0,(a[:,2]+s*u[:,2]+t*v[:,2])[inside].max(initial=0)))
    return np.array(heights)

stations=np.arange(0,85.01,.25)
lanes=[18.25,23.95,29.65,-18.25,-23.95,-29.65]
fig,ax=plt.subplots(2,1,sharex=True,figsize=(9,5.5),layout='constrained')
with (here/'dock-profile.csv').open('w') as out:
    wr=csv.writer(out); wr.writerow(['lateral_mm','station_mm','mesh_z_mm','collision_z_mm'])
    for lane in lanes:
        cad=section(mesh,lane);sim=section(boxes,lane);diff=sim-cad
        wr.writerows(zip(np.full(len(cad),lane),stations,cad,sim))
        if lane==23.95:
            for radius in (10.45,14.5):
                dx=stations[:,None]-stations
                cap=np.sqrt(np.maximum(radius**2-dx**2,0))
                cap[np.abs(dx)>radius]=-np.inf
                error=(sim+cap).max(axis=1)-(cad+cap).max(axis=1)
                center=(stations>=5)&(stations<=75)
                print('Rolling envelope',radius,'mm, signed range',error[center].min(),error[center].max())
            
            ax[0].plot(stations,cad,label='Charger OBJ')
            ax[0].plot(stations,sim,'--',label='MuJoCo collision surface')
        if lane>0:
            ax[1].plot(stations,diff,label=f'{lane:g} mm lateral')
        idx=(stations>=5)&(stations<=80)
        print(lane,'5–80 mm: RMS, min, max',np.sqrt(np.mean(diff[idx]**2)),diff[idx].min(),diff[idx].max(),'largest at',stations[idx][np.argmax(np.abs(diff[idx]))])
ax[0].set_ylabel('Height above table (mm)')
ax[1].set_ylabel('Collision minus OBJ (mm)')
ax[1].set_xlabel('Distance behind dock lip (mm)')
for a in ax:a.legend()
fig.savefig(here/'dock-profile.png',dpi=170)

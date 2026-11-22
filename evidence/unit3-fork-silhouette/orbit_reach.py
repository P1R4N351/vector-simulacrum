"""Fit reach to the 60-degree front edge and check the 195-degree edge."""
import csv
import json
import sys
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from geometry import HERE, mesh

archive = Path(sys.argv[1])
rows = [280,310,340,370,400]
results = []
edge_rows = []
fig,axes = plt.subplots(1,2,figsize=(9,5),layout='constrained')
for ax,heading,side,window in zip(axes,(60,195),(-1,1),((462,510),(157,213))):
    pose=json.loads((HERE.parent/f'shoulder-photo-check/flip/flip-{heading}.json').read_text())
    def project(points):
        q=points@np.array(pose['R']).T+pose['t']
        return q[...,:2]/q[...,2,None]*pose['f']+pose['pp']
    box=project(np.array([[x,y,z] for x in (12,40) for y in (-28,28) for z in (0,45)]))
    a=np.maximum(0,box.min(0).astype(int)-80)
    b=box.max(0).astype(int)+80
    im=cv2.imread(str(archive/f'S-ring{heading}-main.jpg'),cv2.IMREAD_GRAYSCALE)
    roi=im[a[1]:b[1],a[0]:b[0]]
    scale=roi.shape[1]/720
    native_rows=np.round(np.array(rows)*scale).astype(int)
    targets=[]
    for y in native_rows:
        profile=np.mean(roi[y-5:y+6],axis=0)
        gradient=np.gradient(cv2.GaussianBlur(profile.reshape(1,-1),(0,0),3).ravel())
        lo,hi=np.round(np.array(window)*scale).astype(int)
        targets.append(lo+np.argmax(-side*gradient[lo:hi]))
    ax.imshow((roi/255)**.55,cmap='gray',vmin=0,vmax=1)
    ax.scatter(targets,native_rows,marker='+',color='white')
    triangles=mesh[(side*mesh[:,:,1]).min(axis=1)>15]
    def edge(reach,height):
        p=project(triangles+[reach-41+np.sqrt(66**2-(height-45)**2),0,height])-a
        q=np.roll(p,1,axis=1)
        predictions=[]
        for y in native_rows:
            hits=(p[:,:,1]<y)!=(q[:,:,1]<y)
            u,v=p[hits],q[hits]
            x=u[:,0]+(v[:,0]-u[:,0])*(y-u[:,1])/(v[:,1]-u[:,1])
            predictions.append(np.max(x) if heading==60 else np.min(x))
        return np.array(predictions)
    for height in (30,32,34):
        for reach in np.arange(-4,4.01,.25):
            pred=edge(reach,height)
            results.append((heading,height,reach,np.mean(abs(pred-targets))))
    if heading==60:
        selected=min((r for r in results if r[1]==32),key=lambda r:r[3])[2]
    for reach,col in ((0,'orange'),(selected,'cyan')):
        predicted=edge(reach,32)
        ax.plot(predicted,native_rows,color=col)
        for y,measured,model in zip(native_rows,targets,predicted):
            edge_rows.append((heading,reach,y+a[1],measured+a[0],model+a[0]))
    ax.set_title(f'{heading}°');ax.axis('off')
fig.suptitle('Front fork edge · physical picks (+), nominal (orange), 60° fit (cyan)')
fig.savefig(HERE/'orbit-reach.png',dpi=165)
with (HERE/'orbit-reach.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['heading','wrist_height_mm','reach_offset_mm','mean_abs_native_px'])
    w.writerows(results)
print('selected',selected)
for h in (60,195):
    print(h,[r for r in results if r[0]==h and r[1]==32 and r[2] in (0,selected)])

with (HERE/'orbit-reach-picks.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['heading','reach_mm','native_row','measured_column','model_column'])
    w.writerows(edge_rows)

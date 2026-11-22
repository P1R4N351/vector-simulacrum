"""Project the SDK viewer fork and printed fork on the identical measured wrist path."""
import csv
import json
from pathlib import Path
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
import geometry
from low_reach import HERE, ROWS, X, Y


def sdk_mesh(path):
    vertices, faces = [], []
    active = False
    for line in path.read_text().splitlines():
        words = line.split()
        if not words:
            continue
        if words[0] == 'g':
            active = 'fork_geo' in words[1:]
        elif words[0] == 'v':
            vertices.append([float(v) for v in words[1:4]])
        elif active and words[0] == 'f':
            ids = [int(v.split('/')[0])-1 for v in words[1:]]
            faces.extend((ids[0], a, b) for a,b in zip(ids[1:],ids[2:]))
    pose = json.loads(path.with_name('manifest.json').read_text())['viewer_pose']
    px,pz = pose['fork_pivot']
    return 10*(np.asarray(vertices)[faces]-[px,0,pz])


def main():
    stock = sdk_mesh(Path(sys.argv[1]))
    candidates = [('Printed', geometry.mesh, '#ff9f28'), ('SDK', stock, '#00c9cc')]
    poses = json.loads((HERE/'poses.json').read_text())
    observed = np.genfromtxt(HERE/'edges.csv',names=True,delimiter=',',encoding=None,dtype=None)
    records = []
    fig, axes = plt.subplots(1,5,figsize=(12,3.8),layout='constrained',sharey=True)
    for pose,ax in zip(poses,axes):
        trial = pose['trial']
        img = cv2.imread(str(HERE/f'trial{trial}.png'),0)
        ax.imshow(img,cmap='gray',extent=(X-.5,X+149.5,Y+125.5,Y-.5))
        q = observed[(observed['trial']==trial)&(observed['optics']=='shelf')&(observed['reach_mm']==0)]
        ax.scatter(q['observed_x'],q['row'],s=10,color='white',edgecolor='black')
        for name,mesh,color in candidates:
            mask = geometry.silhouette(pose['lift'],pose['head'],np.deg2rad(5.16),-.52, fork_mesh=mesh)
            ax.contour(np.arange(X,X+150),np.arange(Y,Y+126),mask[Y:Y+126,X:X+150],levels=[.5],colors=[color])
            for row,pick in zip(q['row'],q['observed_x']):
                hits=np.flatnonzero(mask[row,500:])
                records.append((trial,name,row,pick,500+hits[0] if len(hits) else np.nan))
        ax.set_title(f'Trial {trial}')
    fig.suptitle('Lowered fork at nominal reach · printed (orange), SDK (teal), physical edge (dots)')
    fig.savefig(HERE/'stock-control.png',dpi=160)
    with (HERE/'stock-control.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['trial','fork','row','observed_x','predicted_x'])
        w.writerows(records)
    with (HERE/'stock-control-raised.csv').open('w') as f:
        w = csv.writer(f,lineterminator='\n')
        w.writerow(['trial','fork','observed_lower_row','predicted_lower_row'])
        for trial,height,observed_row in ((4,67.467987,259),(5,66.160782,272)):
            for name,mesh,_ in candidates:
                mask = geometry.silhouette(height,-.212401658,np.deg2rad(5.16),-.52,fork_mesh=mesh)
                hits = np.flatnonzero(mask[:,570])
                w.writerow([trial,name,observed_row,hits[-1] if len(hits) else np.nan])
    for name,_,_ in candidates:
        for trial in (1,2,3,4,5):
            err=[x-p for t,n,_,p,x in records if t==trial and n==name]
            print(trial,name,np.mean(np.abs(err)),err)

if __name__=='__main__':
    main()

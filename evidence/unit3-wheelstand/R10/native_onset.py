"""Track the top glyph through first motion at the original frame PTS."""
import csv
from pathlib import Path
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
from native_contact import CROPS,HERE
from impact_path import camera


def main(folder):
    with (folder/'frames.csv').open() as f:
        frames=list(csv.DictReader(f))
    results=[]
    fig,ax=plt.subplots(figsize=(8,4.5),layout='constrained')
    for i,(view,(x,y)) in enumerate(zip('AB',[(185,104),(130,102)])):
        project,basis=camera(i)
        left,top,*_=CROPS[i]
        # Restrict to translation-dominated onset, before large yaw or occlusion.
        seq=[r for r in frames if r['view']==view and float(r['dock_s'])<4.42]
        for j,r in enumerate(seq):
            im=cv2.imread(str(folder/r['file']),cv2.IMREAD_GRAYSCALE)
            if j==0:
                tpl=im[y-11:y+12,x-35:x+36]
            roi=im[y-42:y+43,x-76:x+77]
            ncc=cv2.matchTemplate(roi,tpl,cv2.TM_CCOEFF_NORMED)
            _,peak,_,(u,v)=cv2.minMaxLoc(ncc)
            def subpixel(line,k):
                a,b,c=line[k-1:k+2]
                return .5*(a-c)/(a-2*b+c)
            dx=u+subpixel(ncc[v],u)-41
            dy=v+subpixel(ncc[:,u],v)-31
            uv=(np.array([[left+x,top+y],[left+x+dx,top+y+dy]])+.5)*.6-.5
            delta=np.diff(project(uv,44)@basis,axis=0)[0]
            results.append([view,float(r['dock_s']),dx,dy,*delta,np.linalg.norm(delta),peak])
        a=np.array([row[1:] for row in results if row[0]==view])
        ax.plot(a[:,0],a[:,5],'.-',label=f'Camera {view}')
    with (HERE/'native-onset.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['view','dock_s','dx_px','dy_px','forward_mm','left_mm','distance_mm','correlation'])
        w.writerows(results)
    with (HERE/'sim-onset.csv').open() as f:
        for r in csv.DictReader(f):
            color={'A':'tab:blue','B':'tab:orange'}[r['registration']]
            ax.scatter(float(r['first_above_s']),float(r['displacement_mm']),color=color,marker='x',
                       s=65,label='Sim '+r['registration'] if r['displacement_mm']=='1' else None)
    ax.axhline(2,color='gray',ls=':',label='2 mm displacement')
    ax.set(xlabel='Seconds from dock command (camera PTS)',ylabel='Top-glyph displacement (mm)')
    ax.legend()
    fig.savefig(HERE/'native-onset.png',dpi=150)


if __name__=='__main__':
    main(Path(sys.argv[1]))

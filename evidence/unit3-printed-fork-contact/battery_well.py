"""Estimate well center from two references and test a centered axial cell."""
import csv
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
import cv2
import numpy as np
from evaluate import here,run

square=np.float32([[-22,22],[22,22],[22,-22],[-22,-22]])
refs=[
    ('open photo',[(570,181),(956,307),(795,542),(323,354)],
     [(592,321),(601,338),(622,351),(649,360),(675,361),(696,357),(718,346)]),
    ('exploded drawing',[(330,44),(454,62),(457,208),(333,188)],
     [(398,107),(385,105),(377,111),(373,123),(375,135),(381,144),(393,149)])]
with (here/'battery-well.csv').open('w',newline='') as f:
    out=csv.writer(f,lineterminator='\n');out.writerow(('source','u_mm','v_mm','radius_mm','arc_rms_mm'))
    for name,quad,arc in refs:
        H=cv2.getPerspectiveTransform(np.float32(quad),square)
        xy=cv2.perspectiveTransform(np.float32(arc).reshape(-1,1,2),H)[:,0]
        A=np.c_[2*xy,np.ones(len(xy))]
        x,y,c=np.linalg.lstsq(A,np.sum(xy**2,axis=1),rcond=None)[0]
        radius=np.sqrt(c+x*x+y*y)
        out.writerow((name,x,y,radius,np.sqrt(np.mean((np.linalg.norm(xy-[x,y],axis=1)-radius)**2))))
source=Path(sys.argv[2]).resolve()
with tempfile.TemporaryDirectory() as tmp,(here/'battery-axis-replay.csv').open('w',newline='') as f:
    out=csv.writer(f,lineterminator='\n')
    out.writerow(('cell_axis','cell_depth_offset_mm','trial','peak_tilt_deg','rightward_mm','endpoint_error_mm'))
    for axis in range(3):
        for depth in (-6,0,6):
            tree=ET.parse(source)
            inertial=tree.find(".//body[@name='cube']/inertial")
            cell=.009
            total=.050
            radius=.006
            length=.0302
            offset=np.eye(3)[axis]*depth*.001
            diagonal=np.full(3,(total-cell)*.044**2/6+cell*(3*radius**2+length**2)/12)
            diagonal[axis]=(total-cell)*.044**2/6+cell*radius**2/2
            diagonal+=cell*(1-cell/total)*(offset@offset-offset**2)
            inertial.set('pos',' '.join(map(str,offset*cell/total)))
            inertial.set('diaginertia',' '.join(map(str,diagonal)))
            fixture=Path(tmp)/'axial.xml'
            tree.write(fixture)
            sys.argv[2]=str(fixture)
            for trial in range(1,6):
                r,_=run((trial,'stl','overhead','weld',0,0,0),rails=True)
                out.writerow(('xyz'[axis],depth,trial,r[-1],-r[9],r[-3]))

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
fig,axes=plt.subplots(1,2,figsize=(8,4),layout='constrained')
for ax,(name,quad,arc),row in zip(axes,refs,np.genfromtxt(here/'battery-well.csv',delimiter=',',names=True,dtype=None,encoding='utf8')):
    H=cv2.getPerspectiveTransform(np.float32(quad),square)
    points=cv2.perspectiveTransform(np.float32(arc).reshape(-1,1,2),H)[:,0]
    ax.plot(*np.vstack([square,square[0]]).T,color='0.45')
    ax.add_patch(Circle((row[1],row[2]),row[3],fill=False,color='tab:blue'))
    ax.scatter(*points.T,s=16,color='tab:orange',label='Visible rim picks')
    ax.plot(0,0,'k+',label='Face center')
    ax.set(xlabel='Face coordinate (mm)',ylabel='Face coordinate (mm)',title=name,aspect='equal')
axes[0].legend(fontsize=8)
fig.savefig(here/'battery-well.png',dpi=160)

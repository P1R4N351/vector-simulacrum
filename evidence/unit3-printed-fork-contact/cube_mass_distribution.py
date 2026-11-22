"""Sensitivity of the paired shelf pushes to unmeasured cube mass distribution."""
import csv
from itertools import product
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import numpy as np
from evaluate import here,run

source=Path(sys.argv[2]).resolve()
cases=[('uniform',np.zeros(3)),('central insert',np.zeros(3))]
cases.extend(('offset insert',np.array(p)*.014) for p in product((-1,1),repeat=3))
records=[]
with tempfile.TemporaryDirectory() as tmp:
    fixture=Path(tmp)/'mass.xml'
    for grams in (50,85):
        mass=grams*.001
        for name,offset in cases:
            tree=ET.parse(source)
            inertial=tree.find(".//body[@name='cube']/inertial")
            fraction=0 if name=='uniform' else .2
            inertia=np.eye(3)*(1-fraction)*mass*.044**2/6
            inertia+=mass*fraction*(1-fraction)*(offset@offset*np.eye(3)-np.outer(offset,offset))
            inertial.attrib.pop('diaginertia')
            inertial.set('fullinertia',' '.join(map(str,inertia[[0,1,2,0,0,1],[0,1,2,1,2,2]])))
            inertial.set('pos',' '.join(map(str,fraction*offset)))
            inertial.set('mass',str(mass))
            tree.write(fixture)
            sys.argv[2]=str(fixture)
            for trial in range(1,6):
                row,_=run((trial,'stl','overhead','weld',0,0,0),rails=True)
                records.append((grams,name,*(fraction*offset*1000),trial,row[-1],row[-3],row[9]))
with (here/'cube-mass-distribution.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('mass_g','distribution','com_x_mm','com_y_mm','com_z_mm','trial','peak_tilt_deg','endpoint_error_mm','left_mm'))
    w.writerows(records)
fig,axes=plt.subplots(1,2,figsize=(9,4),layout='constrained')
for ax,idx,label in zip(axes,(6,7),('Peak cube tilt (°)','Endpoint error (mm)')):
    for grams,shift,color in [(50,-.12,'#205bb0'),(85,.12,'#a95116')]:
        for trial in range(1,6):
            sub=[r for r in records if r[0]==grams and r[5]==trial]
            ax.scatter([trial+shift]*8,[r[idx] for r in sub[2:]],s=15,color=color,alpha=.5)
            ax.scatter(trial+shift,sub[0][idx],s=65,marker='x',color=color,label=f'{grams} g uniform' if trial==1 else None)
    ax.set(xlabel='Recorded push',ylabel=label,xticks=range(1,6))
axes[0].legend()
fig.savefig(here/'cube-mass-distribution.png',dpi=160)
for grams in (50,85):
    a=np.array([r[-3:] for r in records if r[5]==1 and r[0]==grams])
    print(grams,a[0],a.min(axis=0),a.max(axis=0))

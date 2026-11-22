"""Replay the unmodified cube contact surface in all 24 resting orientations."""
import csv
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from evaluate import here, run

source = Path(sys.argv[2]).resolve()
rotations = np.rint(Rotation.create_group('O').as_matrix()).astype(int)
axes = np.array(['x','y','z'])
def axis_name(row):
    i = np.flatnonzero(row)[0]
    return ('+' if row[i]>0 else '-') + axes[i]

records = []
with tempfile.TemporaryDirectory() as folder:
    fixture = Path(folder)/'oriented.xml'
    for R in rotations:
        tree = ET.parse(source)
        for geom in tree.findall(".//body[@name='cube']/geom"):
            assert geom.get('type')=='box'
            for field, matrix in [('pos',R), ('size',abs(R))]:
                vec = np.fromstring(geom.get(field),sep=' ')
                geom.set(field,' '.join(map(str,matrix@vec)))
        tree.write(fixture)
        sys.argv[2] = str(fixture)
        for trial in range(1,6):
            result,_ = run((trial,'stl','overhead','weld',0,0,0),rails=True)
            records.append((axis_name(R[2]),axis_name(R[0]),trial,result[-1],result[-3],result[9]))

with (here/'face-orientation.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('upward_original_axis','forward_original_axis','trial','peak_tilt_deg','endpoint_error_mm','left_mm'))
    w.writerows(records)
fig,ax=plt.subplots(1,2,figsize=(9,4),layout='constrained')
for i,(axis,metric,label) in enumerate(zip(ax,(3,4),('Peak tilt (°)','Endpoint error (mm)'))):
    for j,up in enumerate(sorted(set(row[0] for row in records))):
        for trial in range(1,6):
            values=[row[metric] for row in records if row[0]==up and row[2]==trial]
            xs=trial+(j-2.5)*.08+np.linspace(-.018,.018,4)
            axis.scatter(xs,values,s=13,color=f'C{j}',label=up if trial==1 else None)
    axis.set(xlabel='Recorded push',ylabel=label,xticks=range(1,6))
ax[1].legend(title='Up axis',fontsize=8)
fig.savefig(here/'face-orientation.png',dpi=160)
for trial in range(1,6):
    subset=np.array([r[3:] for r in records if r[2]==trial])
    print(trial,subset.min(axis=0),subset.max(axis=0))

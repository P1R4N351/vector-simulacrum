"""Index native camera exposures against robot feedback and the seven optical sightings."""
import csv
import gzip
import re
import sys
from pathlib import Path
import zipfile

import cv2
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent/'D'
raw=Path(sys.argv[1])
start=float(re.search(r'\{"t": ([0-9.]+), "kind": "begin", "action": "undock"',(raw/'cube-repeat-D.jsonl').read_text()).group(1))
with gzip.open(here/'robot.csv.gz','rt') as f:robot=np.genfromtxt(f,names=True,delimiter=',')
objects=np.genfromtxt(here/'object-events.csv',delimiter=',',names=True,dtype=None,encoding=None)
files=sorted((raw/'p2-offset-D-frames').glob('*.jpg'))
def times(f):
    host,robot_ms,seq=f.stem.split('-')
    return float(host)-start,int(robot_ms),int(seq)
selected=[f for f in files if 53<=times(f)[0]<=81]
with zipfile.ZipFile(here/'native-views.zip','w') as archive,(here/'native-index.csv').open('w') as stream:
    writer=csv.writer(stream,lineterminator='\n')
    writer.writerow(['file','camera_receipt_s','robot_frame_ms','state_minus_camera_receipt_s','head_deg','yaw_deg'])
    for i,f in enumerate(selected):
        t,clock,_=times(f);r=robot[np.argmin(abs(robot['receipt_s']-t))]
        label=f'frame-{i:03}.jpg'
        archive.write(f,label)
        writer.writerow([label,round(t,5),clock,round(r['receipt_s']-t,4),
                         round(np.degrees(r['head_rad']),2),round(np.degrees(r['yaw_rad']),2)])
fig,axes=plt.subplots(2,3,figsize=(9,4.9),layout='constrained')
for ax,s in zip(axes.flat,objects[[0,1,2,3,4,6]]):
    f=min(selected,key=lambda p:abs(times(p)[0]-s['receipt_s']))
    im=cv2.imread(str(f),cv2.IMREAD_GRAYSCALE)
    ax.imshow(im,cmap='gray',vmin=0,vmax=255);ax.set_title(f"{times(f)[0]:.2f} s");ax.axis('off')
fig.suptitle('Onboard views nearest the cube detections by host receipt time')
fig.savefig(here/'native-sightings.jpg',dpi=150)
print(len(selected),im.shape)

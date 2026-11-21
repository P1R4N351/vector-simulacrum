"""Track the original timestamped top-glyph crops; command accepts a directory or the published ZIP."""
import csv
from pathlib import Path
import sys
import zipfile

import cv2
import numpy as np

out=Path(__file__).resolve().parent/'E'
source=Path(sys.argv[1])
epoch=1795299509.3353744  # native undock begin, pod Unix seconds
if source.is_dir():
    with zipfile.ZipFile(out/'onset-crops.zip','w') as z:
        for p in sorted(source.glob('*.jpg')):
            if 45<float(p.stem)-epoch<53:
                roi=cv2.imread(str(p))[207:360,106:313]
                ok,buf=cv2.imencode('.png',roi); assert ok
                z.writestr(p.stem+'.png',buf.tobytes())
    source=out/'onset-crops.zip'
# Outer white border in the first crop (TL, TR, BR, BL); nominal 23 mm.
quad=np.float32([[80,33],[129,42],[105,77],[49,64]])
dest=np.float32([[80,80],[149,80],[149,149],[80,149]])
warp=cv2.getPerspectiveTransform(quad,dest)
with zipfile.ZipFile(source) as z:
    frames=[(float(Path(n).stem)-epoch,cv2.warpPerspective(cv2.imdecode(np.frombuffer(z.read(n),np.uint8),0),warp,(220,210))) for n in sorted(z.namelist())]
results=[]
# Search position AND rotation jointly to avoid interpreting rotation as translation.
patch=frames[0][1][82:147,82:147]
templates=[]
for deg in np.arange(-8,32.1,.5):
    rot=cv2.getRotationMatrix2D((32,32),deg,1.)
    templates.append((deg,cv2.warpAffine(patch,rot,(65,65))[13:52,13:52]))
for t,img in frames:
    score,angle,x,y=max((s,angle,x,y) for angle,tmp in templates
                        for _,s,_,(x,y) in [cv2.minMaxLoc(cv2.matchTemplate(img[52:168,43:160],tmp,cv2.TM_CCOEFF_NORMED))])
    results.append([t,(x+43+19-114)/3,(y+52+19-114)/3,angle,score])
with (out/'onset-camera.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n');w.writerow(['receipt_s','x_mm','y_mm','yaw_deg','correlation']);w.writerows(results)

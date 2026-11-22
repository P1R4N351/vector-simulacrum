"""Estimate cube relative to front axle from a single upright precontact frame."""
import csv
from pathlib import Path
import sys
import cv2
import numpy as np
from scipy.optimize import least_squares

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[2]))
from simulacrum.camera_spins import WORLD, calibration, intersect, project, LENS_HEIGHT_MM, LENS_X_MM

offset=np.array([380,470])
dock=np.array([[430,89],[501,89],[501,157],[431,157]])+offset
# Cube top glyph centre, forward end of backpack lens, two ends of raised fork.
picks=np.array([[271,203],[410,213],[316,128],[380,110]])+offset
def estimate(focal, lens_height, fork_height, uv):
    def residual(p):
        return (project(WORLD[:4],np.r_[p,focal])-dock).ravel()
    fit=least_squares(residual,calibration()[:6])
    camera=np.r_[fit.x,focal]
    cube=intersect(uv[:1],44,camera)[0]
    lens=intersect(uv[1:2],lens_height,camera)[0]
    a,b=intersect(uv[2:],fork_height,camera)
    cross=b-a
    forward=np.array([-cross[1],cross[0]])/np.linalg.norm(cross)
    roof_ahead=intersect(np.array([[352,181]])+offset,lens_height,camera)[0]
    if forward@(roof_ahead-lens)<0:forward=-forward
    left=np.array([-forward[1],forward[0]])
    delta=cube-lens
    return [delta@forward+LENS_X_MM,delta@left,np.sqrt(np.mean(residual(fit.x)**2))]
im=cv2.imread(str(here/'precontact-crop.jpg'))
for uv in dock-offset:cv2.circle(im,tuple(uv),3,(0,180,255),1)
for uv,label in zip(picks-offset,['cube','lens','fork A','fork B']):
    cv2.circle(im,tuple(uv),4,(0,255,140),1)
    cv2.putText(im,label,tuple(uv+[4,-5]),0,.4,(0,255,140),1,cv2.LINE_AA)
cv2.imwrite(str(here/'camera-picks.jpg'),im)
with (here/'camera-pose.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['case','focal_px','lens_height_mm','fork_height_mm','forward_mm','left_mm','dock_rms_px_per_coordinate'])
    for focal in (492.9,498.4,504.5):
        for lens in (63.5,LENS_HEIGHT_MM,67.5):
            for fork in (85,93,101):
                w.writerow(['geometry',focal,lens,fork,*estimate(focal,lens,fork,picks)])
    for i in range(4):
        for axis in (0,1):
            for step in (-3,3):
                uv=picks.copy();uv[i,axis]+=step
                w.writerow([f'pick {i} axis {axis} delta {step}',498.4,LENS_HEIGHT_MM,93,*estimate(498.4,LENS_HEIGHT_MM,93,uv)])

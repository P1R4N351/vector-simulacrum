"""Measure image-plane rotation independently on opposite cube faces."""
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

from face_contact import start_raise

root = Path(__file__).resolve().parent
video = cv2.VideoCapture(str(root/'docked-contact.mp4'))
fps = video.get(cv2.CAP_PROP_FPS)
regions = {'A':(332,266,63,62), 'B':(552,214,65,59)}
points = {}
rows = []
fig, axes = plt.subplots(2,2,figsize=(9,6),layout='constrained',height_ratios=[1.2,1])
for frame in range(int(video.get(cv2.CAP_PROP_FRAME_COUNT))):
    ok, image = video.read()
    assert ok
    t = frame/fps-.5
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    if frame == 0:
        ref = gray.copy()
        for view,(x,y,w,h) in regions.items():
            mask = np.zeros_like(gray)
            mask[y:y+h,x:x+w]=255
            points[view] = cv2.goodFeaturesToTrack(ref,70,.02,3,mask=mask)
    for j,(view,src) in enumerate(points.items()):
        dst,good,_ = cv2.calcOpticalFlowPyrLK(ref,gray,src,None)
        back,returned,_ = cv2.calcOpticalFlowPyrLK(gray,ref,dst,None)
        keep = (good.ravel()!=0)&(returned.ravel()!=0)&(np.linalg.norm(back-src,axis=2).ravel()<.65)
        a,b = src[keep,0], dst[keep,0]
        fit,inlier = cv2.estimateAffinePartial2D(a,b,method=cv2.RANSAC,ransacReprojThreshold=1.)
        assert fit is not None
        a,b = a[inlier.ravel()!=0], b[inlier.ravel()!=0]
        assert len(a)>=5
        dy,dx = fit[1,0],fit[0,0]
        angle = np.degrees(np.arctan2(dy,dx))
        error = np.linalg.norm(a@fit[:,:2].T+fit[:,2]-b,axis=1)
        # Compare long-baseline feature pairs to avoid unstable short vectors.
        i,k = np.triu_indices(len(a),1)
        u,v = a[k]-a[i],b[k]-b[i]
        long = np.linalg.norm(u,axis=1)>20
        angles = np.degrees(np.arctan2(u[:,0]*v[:,1]-u[:,1]*v[:,0],(u*v).sum(axis=1)))[long]
        lo,hi = np.percentile(angles,[10,90])
        rows.append([t,view,len(a),angle,lo,hi,np.sqrt(np.mean(error**2))])
        if abs(t-1.35)<.001:
            x,y,w,h = regions[view]
            ax=axes[0,j]
            ax.imshow(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
            ax.quiver(a[:,0],a[:,1],b[:,0]-a[:,0],b[:,1]-a[:,1],angles='xy',scale_units='xy',scale=1,color='orange',width=.006)
            ax.set(xlim=(x-20,x+w+20),ylim=(y+h+15,y-30),title=f'Camera {view} · +1.35 s')
            ax.axis('off')
video.release()
for j,view in enumerate(regions):
    m=np.array([[r[0],*r[2:]] for r in rows if r[1]==view])
    t,count,angle,lo,hi,error=m.T
    ax=axes[1,j]
    ax.fill_between(t,lo,hi,color='slategray',alpha=.3,label='Long feature pairs, 10–90%')
    ax.plot(t,angle,label='Face rotation')
    ax.axvspan(1.35,1.45,alpha=.13,color='orange')
    ax.set(xlabel='Seconds after command',ylabel='Image rotation (degrees)',ylim=(-7,7))
    if j==0: ax.legend(fontsize=8)
    for window in [(-.5,0),(1.1,1.36),(1.44,1.46),(1.8,2.4)]:
        selected=(t>=window[0])&(t<window[1])
        print(view,window,'angle min/max',angle[selected].min(),angle[selected].max(),'features min',min(count[selected]))
fig.savefig(root/'release-rotation.png',dpi=165)
with (root/'release-rotation.csv').open('w') as stream:
    writer=csv.writer(stream)
    writer.writerow('seconds view inliers image_deg pair_p10_deg pair_p90_deg fit_rms_px'.split())
    writer.writerows(rows)

# Independent 3-D attitude check: mean force directions in the raised and
# settled windows. Dynamics can bias the physical inclination estimate.
samples=np.loadtxt(root/'cube.csv',delimiter=',',skiprows=1)
reference=samples[samples[:,0]<0,2:].mean(axis=0)
reference/=np.linalg.norm(reference)
checks=[]
for lo,hi in [(1.10,1.35),(1.20,1.35),(1.74,2.24)]:
    window=samples[(samples[:,0]>=lo)&(samples[:,0]<hi),2:]/8192
    mean=window.mean(axis=0)
    angle=np.degrees(np.arccos(np.clip(mean@reference/np.linalg.norm(mean),-1,1)))
    checks.append(['Physical mean force',lo,hi,len(window),angle])
for gap in (0,1,2):
    sim,_=start_raise(gap)
    body=sim.data.body('cube')
    initial=body.xmat.reshape(3,3)[:,2].copy()
    vectors=[]
    for _ in range(round(2.25/sim.dt)):
        sim.step()
        vectors.append(body.xmat.reshape(3,3)[:,2].copy())
    vectors=np.array(vectors)
    clock=(np.arange(len(vectors))+1)*sim.dt
    tilt=np.degrees(np.arccos(np.clip(vectors@initial,-1,1)))
    for lo,hi in [(0,2.25),(1.10,1.35),(1.74,2.24)]:
        mask=(clock>=lo)&(clock<hi)
        checks.append([f'{gap} mm gap max tilt',lo,hi,int(mask.sum()),tilt[mask].max()])
with (root/'release-inclination.csv').open('w') as stream:
    writer=csv.writer(stream)
    writer.writerow(['quantity','start_s','end_s','samples','degrees'])
    writer.writerows(checks)

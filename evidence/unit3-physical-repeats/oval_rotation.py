"""Measure changes in shelf-plane heading from the outer white square on the cube."""
import csv
from pathlib import Path
import cv2
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
out = here/'E'
video = cv2.VideoCapture(str(out/'overhead.mp4'))
track = np.genfromtxt(out/'glyph-track.csv', delimiter=',', names=True)
square = np.float32([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]])
taps = [float(e['receipt_s']) for e in csv.DictReader((out/'object-events.csv').open()) if e['event']=='object_tapped']

def corners(frame, center, threshold):
    x,y = np.int32(center)
    origin = [x-26,y-21]
    patch = frame[y-21:y+22,x-26:x+27]
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    contours,_ = cv2.findContours((gray>threshold).astype('uint8'),cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
    for contour in sorted(contours,key=cv2.contourArea,reverse=True):
        area = cv2.contourArea(contour)
        if not 620 < area < 1000:
            continue
        pts = contour[:,0].astype(float)
        if np.any(pts.min(axis=0)==0) or np.any(pts.max(axis=0)==[52,42]):
            continue
        # Fit straight portions of the four sides, excluding rounded molded corners.
        q = pts[np.argmax(pts @ square.T,axis=0)]
        for _ in range(3):
            edge = np.roll(q,-1,axis=0)-q
            rel = pts[:,None,:]-q
            frac = (rel*edge).sum(axis=2)/(edge**2).sum(axis=1)
            dist = np.abs(rel[:,:,0]*edge[:,1]-rel[:,:,1]*edge[:,0])/np.linalg.norm(edge,axis=1)
            lines = []
            for i in range(4):
                use = (dist.argmin(axis=1)==i)&(frac[:,i]>.17)&(frac[:,i]<.83)
                if use.sum()<5: break
                p = pts[use]
                _,_,v = np.linalg.svd(p-p.mean(axis=0),full_matrices=False)
                normal = v[-1]
                lines.append([*normal,-normal@p.mean(axis=0)])
            if len(lines)!=4:break
            h = np.cross(np.roll(lines,1,axis=0),lines)
            if np.min(abs(h[:,2]))<.25:break
            q = h[:,:2]/h[:,2,None]
        else:
            return (q+origin).astype(np.float32)

def measure(ref, quad):
    matrix=cv2.getPerspectiveTransform(ref,square)
    rect=cv2.perspectiveTransform(quad[None],matrix)[0]
    # Best-fit 2D rotation after rectification; translation cancels on centering.
    z=(rect[:,0]+1j*rect[:,1]); basis=square[:,0]+1j*square[:,1]
    return -np.angle(np.vdot(basis,z),deg=True)

fig,axes=plt.subplots(4,2,figsize=(6.4,8.5),layout='constrained')
rows=[]
controls = [71.,75.,79.,83.]
for idx,tap in enumerate(taps+controls):
    # Median over two settled windows and three segmentation levels.
    windows = [[tap+s for s in (-1.6,-1.4,-1.2,-1.,-.8)], [tap+s for s in (.8,1.,1.2,1.4,1.6)]]
    angles=[]
    ref_quads=[]
    for thresh in (180,195,210):
        ends=[]
        for j,times in enumerate(windows):
            fits=[]
            for t in times:
                k=int(round(t*5))
                video.set(cv2.CAP_PROP_POS_FRAMES,k)
                ok,frame=video.read()
                assert ok
                center=[track[k]['x_px'],track[k]['y_px']]
                q=corners(frame,center,thresh)
                if q is None:continue
                fits.append(q)
                if idx<len(taps) and thresh==195 and len(fits)==1:
                    cv2.polylines(frame,[np.int32(np.round(q))],True,(0,220,80),1,cv2.LINE_AA)
                    axes[idx,j].imshow(frame[125:231,69:205,::-1])
                    axes[idx,j].set_title(f'{k/5:.1f} s');axes[idx,j].axis('off')
            if len(fits)>=3: ends.append(np.median(fits,axis=0))
        if len(ends)==2:
            angles.append(measure(*ends));ref_quads.append(ends[0])
    if len(angles)<2:
        assert idx>=len(taps)
        rows.append(["control",tap,len(angles),None,None,None,None,None])
        continue
    def median_patch(times):
        images=[]
        for t in times:
            k=round(t*5);video.set(cv2.CAP_PROP_POS_FRAMES,k);ok,frame=video.read();assert ok
            # Local foreshortening is set by the average opposing edges of the first glyph.
            x,y=track[k]['x_px'],track[k]['y_px']
            mat=cv2.getAffineTransform(np.float32([[x,y],[x,y]+right,[x,y]+down]),np.float32([[40,40],[100,40],[40,100]]))
            g=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            images.append(cv2.warpAffine(g,mat,(80,80)))
        return np.median(images,axis=0).astype(np.uint8)
    baseline=ref_quads[len(ref_quads)//2]
    right=((baseline[1]-baseline[0])+(baseline[2]-baseline[3]))/2
    down=((baseline[3]-baseline[0])+(baseline[2]-baseline[1]))/2
    before,after=map(median_patch,windows)
    mask=np.zeros((80,80),np.uint8);cv2.circle(mask,(40,40),30,255,-1)
    quality,rotation=cv2.findTransformECC(before,after,np.eye(2,3,dtype=np.float32),cv2.MOTION_EUCLIDEAN,
                                         (cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,150,1e-6),mask)
    yaw=-np.rad2deg(np.arctan2(rotation[1,0],rotation[0,0]))
    rows.append(['tap' if idx<len(taps) else 'control',tap,len(angles),*np.round([np.median(angles),min(angles),max(angles),yaw,quality],3)])
fig.suptitle('Cube top glyph before and after each tap')
fig.savefig(out/'tap-heading.png',dpi=150)
with (out/'tap-heading.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['window_type','center_s','thresholds_used','ccw_change_deg','min_threshold_deg','max_threshold_deg','texture_ccw_deg','texture_correlation'])
    w.writerows(rows)
print(rows)

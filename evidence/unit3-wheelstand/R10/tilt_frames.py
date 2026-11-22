"""Estimate the inclination of the same top glyph while the cube turns."""
import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt
from planar_slide import HERE, K

# Approximate top-glyph centers in the corrected 600 px views; B is occluded from frame 78.
CENTERS = {"A": [(442,143),(442,143),(455,148),(458,150),(459,153),(464,159),
                 (464,162),(465,163),(468,164),(466,166),(465,167),(465,168)]+[(462,168)]*16,
           "B": [(168,106),(168,106),(166,104),(170,101)]}
SQUARE = np.array([[-1,-1,0],[1,-1,0],[1,1,0],[-1,1,0]],float)*11.5


def corners(gray,center,threshold):
    x,y=center
    cut=gray[y-24:y+24,x-43:x+43]
    contours,_=cv2.findContours(np.uint8(cut>threshold)*255,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    choices=[]
    for c in contours:
        area=cv2.contourArea(c)
        if not 75 < area < 1000:
            continue
        hull=cv2.convexHull(c)
        poly=cv2.approxPolyDP(hull, .035*cv2.arcLength(hull,True),True)
        if len(poly)!=4:
            continue
        points=poly[:,0,:].astype(float)
        if np.min(points)<=0 or np.max(points[:,0])>=85 or np.max(points[:,1])>=47:
            continue
        err=np.linalg.norm(points.mean(axis=0)-[43,24])
        if err < 9:
            choices.append((err,points+[x-43,y-24]))
    if not choices:
        return None
    return min(choices,key=lambda item:item[0])[1]


# The upright edges of the visible side walls run approximately up in these
# crops. Use this qualitative cue only to resolve the two planar pose branches.
def normal(points):
    ok,rotations,translations,_=cv2.solvePnPGeneric(SQUARE,points,K,None,flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    choices=[]
    unit=np.array([0.,-1.])
    for r,t in zip(rotations,translations):
        n=cv2.Rodrigues(r)[0][:,2]
        xyz=t.ravel()
        if n@xyz>0:
            n=-n
        segment=np.array([xyz,xyz+8*n])@K.T
        uv=segment[:,:2]/segment[:,2:]
        direction=uv[1]-uv[0]
        agreement=unit@direction/np.linalg.norm(direction)
        choices.append((agreement,n))
    score,n=max(choices,key=lambda item:item[0])
    return n if score>np.cos(np.radians(25)) else None


def angle(a,b):
    return np.degrees(np.arccos(np.clip(a@b,-1,1)))


def main():
    cap=cv2.VideoCapture(str(HERE/'paired.mp4'))
    results=[]
    fig,axes=plt.subplots(2,4,figsize=(10,5),layout='constrained')
    tiles={("A",f):i for i,f in enumerate([74,78,81,88,90,99])}
    tiles.update({("B",75):6,("B",77):7})
    for view,centers in CENTERS.items():
        baseline={}
        for idx,center in enumerate(centers):
            frame=74+idx
            cap.set(cv2.CAP_PROP_POS_FRAMES,frame)
            ok,image=cap.read()
            assert ok
            x0=0 if view=='A' else 600
            image=image[:,x0:x0+600]
            gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            estimates=[]
            for threshold in range(80,191,10):
                pts=corners(gray,center,threshold)
                if pts is None:
                    continue
                n=normal(pts)
                if n is None:
                    continue
                if frame==74:
                    baseline[threshold]=n
                if threshold in baseline:
                    estimates.append((angle(n,baseline[threshold]),pts))
            if not estimates:
                continue
            values=np.array([e[0] for e in estimates])
            median=np.median(values)
            pts=estimates[np.argmin(abs(values-median))][1]
            results.append([view,frame,frame/12-2,len(values),median,*np.percentile(values,[10,90])])
            if (view,frame) in tiles:
                ax=axes.flat[tiles[view,frame]]
                ax.imshow(image[:,:,::-1]);ax.plot(*np.vstack([pts,pts[0]]).T,'c-',lw=.8)
                x,y=center;ax.set(xlim=(x-43,x+43),ylim=(y+32,y-32),title=f'{view} · {frame/12-2:.2f} s')
    for ax in axes.ravel(): ax.axis('off')
    fig.savefig(HERE/'tilt-picks.jpg',dpi=140)
    cap.release()
    with (HERE/'tilt-frames.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['view','frame','dock_s','thresholds','median_deg','threshold_p10_deg','threshold_p90_deg'])
        w.writerows(results)


if __name__=='__main__':
    main()

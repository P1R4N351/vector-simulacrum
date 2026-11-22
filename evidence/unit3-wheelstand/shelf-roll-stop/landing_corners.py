"""Extract intersections of the four straight outer glyph edges from corrected native crops."""
import csv
import sys
from pathlib import Path
import cv2
import numpy as np

here = Path(__file__).resolve().parent

def corners(gray, cutoff):
    candidates=[]
    for c in cv2.findContours(np.uint8(gray > cutoff), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[0]:
        x,y,w,h=cv2.boundingRect(c)
        if not (88<x<145 and 30<y<110 and 38<w<70 and 32<h<65): continue
        if cv2.contourArea(c, oriented=True) >= 0: continue
        if not (1250 < cv2.contourArea(c) < 3200): continue
        pts=c[:,0].astype(float)
        box=cv2.boxPoints(cv2.minAreaRect(c)).astype(float)
        box=np.roll(box,-np.argmin(box.sum(axis=1)),axis=0)
        if box[1,0]<box[-1,0]: box=box[[0,3,2,1]]
        lines=[]
        for a,b in zip(box,np.roll(box,-1,axis=0)):
            along=(b-a)/np.linalg.norm(b-a)
            normal=np.array([-along[1],along[0]])
            t=(pts-a)@along/np.linalg.norm(b-a)
            use=(t>.2)&(t<.8)&(np.abs((pts-a)@normal)<6)
            if use.sum()<10: break
            sel=pts[use]; n=np.linalg.svd(sel-sel.mean(axis=0),full_matrices=False)[2][-1]
            lines.append((n,n@sel.mean(axis=0)))
        if len(lines)!=4: continue
        quad=[]
        for i in range(4):
            A,b=zip(lines[i-1],lines[i])
            quad.append(np.linalg.solve(A,b))
        candidates.append((cv2.contourArea(c),np.array(quad)))
    if candidates: return max(candidates,key=lambda item:item[0])[1]

if __name__=='__main__':
    data=np.load(sys.argv[1])
    with (here/'landing-corners.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['t','threshold',*[f'{k}{i}' for i in range(4) for k in 'xy']])
        for t,im in zip(data['t'],data['im']):
            if not (-.8<t<-.1 or 4.95<t<5.60 or 6<t<7): continue
            for threshold in (80,90,100):
                q=corners(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),threshold)
                if q is not None:w.writerow([round(t,6),threshold,*np.round(q.ravel(),3)])

"""Local pinhole cross-validation on three adjacent glyphs on a stationary cube."""
import csv
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
# Seeds label each corner. Robust line fits use the middle of each outer edge,
# avoiding the rounded printed corners. Coordinates refer to the published crop.
seeds = np.array([[(112,29),(164,45),(215,33),(162,19)],
                  [(80,71),(131,92),(119,163),(67,144)],
                  [(190,91),(237,77),(226,147),(176,164)]], float)
lo, hi = (44-23)/2, (44+23)/2
world = np.array([[(hi,lo,44),(lo,lo,44),(lo,hi,44),(hi,hi,44)],
                  [(hi,0,hi),(lo,0,hi),(lo,0,lo),(hi,0,lo)],
                  [(0,lo,hi),(0,hi,hi),(0,hi,lo),(0,lo,lo)]],float)
world = world[:, :, [1, 0, 2]].copy()  # outward right, outward left, up: right-handed

names = ["top","left","right"]


def corners(gray, seed, threshold):
    contours,_ = cv2.findContours(np.uint8(gray>threshold),cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
    cx,cy=seed.mean(axis=0)
    contour=max((c for c in contours if cv2.pointPolygonTest(c,(cx,cy),False)>=0), key=cv2.contourArea)
    points=contour[:,0,:]
    lines=[]
    for a,b in zip(seed,np.roll(seed,-1,axis=0)):
        u=(b-a)/np.linalg.norm(b-a)
        along=(points-a)@u/np.linalg.norm(b-a)
        edge=points[(along>.2)&(along<.8)&(abs((points-a)@[-u[1],u[0]])<3.5)]
        assert len(edge)>=6
        vx,vy,x,y=cv2.fitLine(edge,cv2.DIST_HUBER,0,.01,.01).ravel()
        lines.append(np.array([-vy,vx,vy*x-vx*y]))
    xs=np.cross(np.roll(lines,1,axis=0),lines)
    return xs[:,:2]/xs[:,2:]


def project(points,p):
    if len(p)==8:
        # Move each face along its normal, preserving its 23 mm square.
        points=points.copy()
        points[points==0] -= p[7]/2
        points[points==44] += p[7]/2
    rot=cv2.Rodrigues(p[:3])[0]
    cam=points.reshape(-1,3)@rot.T+p[3:6]
    return cam[:,:2]/cam[:,2:] * np.exp(p[6]) + [-80.5,-136.5]


def fit(x,y,flex):
    # Recover pose for each starting focal length and refine with square pixels.
    candidates=[]
    for f in [400.,800.,1400.]:
        k=np.array([[f,0,-80.5],[0,f,-136.5],[0,0,1.]])
        ok,r,t=cv2.solvePnP(x,y,k,None,flags=cv2.SOLVEPNP_SQPNP)
        if ok:
            start=np.r_[r.ravel(),t.ravel(),np.log(f)]
            if flex:
                start=np.r_[start,0.]
            result=least_squares(lambda p:(project(x,p)-y).ravel(),start,
                                 bounds=([-np.inf]*6+[np.log(100)]+([-12] if flex else []), [np.inf]*6+[np.log(5000)]+([12] if flex else [])))
            candidates.append(result)
    return min(candidates,key=lambda r:r.cost).x


def dlt(x,y,target):
    h=np.column_stack((x,np.ones(len(x))))
    zero=np.zeros_like(h)
    design=np.vstack((np.column_stack((h,zero,-y[:,0,None]*h)),
                      np.column_stack((zero,h,-y[:,1,None]*h))))
    matrix=np.linalg.svd(design)[2][-1].reshape(3,4)
    uv=np.column_stack((target,np.ones(len(target))))@matrix.T
    return uv[:,:2]/uv[:,2:]


def main():
    img=cv2.imread(str(HERE/"source.png"))
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    overlay=cv2.resize(img,None,fx=3,fy=3)
    rows=[]
    for th in (100,145,170):
        obs=np.array([corners(gray,s,th) for s in seeds])
        for i,name in enumerate(names):
            keep=np.arange(3)!=i
            x=world[keep].reshape(-1,3)
            y=obs[keep].reshape(-1,2)
            uv=dlt(x,y,world[i])
            rows.append(dict(threshold=th,held_out=name,model="free projection",spacing_mm=44,focal_px="",
                             fit_rms_px=np.sqrt(np.mean(np.sum((dlt(x,y,x)-y)**2,axis=1))),
                             heldout_rms_px=np.sqrt(np.mean(np.sum((uv-obs[i])**2,axis=1)))))
            for flex in (False,True):
                p=fit(world[keep].reshape(-1,3),obs[keep].reshape(-1,2),flex)
                xy=project(world[i],p)
                residual=xy-obs[i]
                rows.append(dict(threshold=th,held_out=name,model="free spacing" if flex else "nominal",spacing_mm=44+p[7] if flex else 44,focal_px=np.exp(p[6]),
                                 fit_rms_px=np.sqrt(np.mean(np.sum((project(world[keep],p)-obs[keep].reshape(-1,2))**2,axis=1))),
                                 heldout_rms_px=np.sqrt(np.mean(np.sum(residual**2,axis=1)))))
                if th==145 and not flex:
                    for pts,color in [(obs[i],(50,230,60)),(xy,(210,70,230)),(uv,(0,155,255))]:
                        cv2.polylines(overlay,[np.int32(np.rint(pts*3))],True,color,2,cv2.LINE_AA)
    with (HERE/"face-transfer.csv").open("w") as stream:
        w=csv.DictWriter(stream,fieldnames=rows[0],lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    cv2.imwrite(str(HERE/"face-transfer.jpg"),overlay)


if __name__=="__main__":
    main()

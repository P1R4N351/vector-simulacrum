from pathlib import Path
import sys,json
import numpy as np
import cv2 as cv
from scipy.ndimage import map_coordinates,gaussian_filter1d
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import os
reference=Path(os.environ.get('CAP_SCALE_DIR', Path(__file__).resolve().parents[2]/'orbit-cap-scale'))
sys.path.insert(0,str(reference))
from fit import fit,FOCAL,CENTER,residual
raw=Path(os.environ.get('FLIP_ARCHIVE',Path.home()/'Pictures/Vector/P2Y8-20261119-three-camera'))
seeds={105:[(575,539,26,56,.12),(440,606,32,52,.16)],180:[(222,545,22,60,-.05),(331,612,26,54,-.1)]}
def edges(im,seed):
    x,y,w,h,s=seed
    theta=np.arange(0,2*np.pi,2*np.pi/96); vec=np.array([w*np.cos(theta)-h*np.sin(-s)*np.sin(theta),h*np.sin(theta)])
    # Tip of each semi-axis follows the inclined face (right at top for positive s).
    vec[0]=w*np.cos(theta)-s*h*np.sin(theta)
    paths=8*(np.array([x,y])[None,:]+vec.T)
    norm=np.array([np.cos(theta)/w,np.sin(theta)/h+s*np.cos(theta)/w]).T;norm/=np.linalg.norm(norm,axis=1)[:,None]
    ds=np.arange(-50,51)
    b,g,r=cv.split(im);ls=cv.GaussianBlur(b.astype(np.float32)-g.astype(np.float32),(0,0),3)
    for k in range(3):
        samp=map_coordinates(ls,(paths[:,None,:]+ds[None,:,None]*norm[:,None,:]).transpose(2,0,1)[::-1],order=1)
        strength=-np.gradient(samp,axis=1);n=np.argmax(strength,axis=1)
        good=(strength[np.arange(96),n]>.12)&(abs(ds[n])<47)
        found=(paths+ds[n,None]*norm)
        e=cv.fitEllipseAMS(np.float32(found[good]));ell=ellpoly(e)
        # Rejection by angular-sector continuity, distance from consensus fitted ellipse
        delta=np.linalg.norm(ell[:,None,:]-found[None,:,:],axis=2).min(0)
        good&=delta<16
        if good.sum()<45:raise RuntimeError((seed,good.sum()))
        e=cv.fitEllipseAMS(np.float32(found[good]));ell=ellpoly(e)
        if k<2:paths=ell[np.linalg.norm(ell[:,None,:]-paths[None,:,:],axis=2).argmin(0)]
    return found[good],e
def ellpoly(e):
    ctr,wh,ang=e; t=np.arange(720)*np.pi/360;rr=np.deg2rad(ang)
    return np.column_stack([np.cos(t),np.sin(t)])*(np.array(wh)/2)@np.array([[np.cos(rr),np.sin(rr)],[-np.sin(rr),np.cos(rr)]])+ctr
def world(p,sgn):
    side=-1 if sgn in (105,60) else 1
    d=np.array([-48,0,-3.75])/np.hypot(48,3.75);up=np.array([d[2],0,-d[0]])
    for su in [-1,1]:
        A=np.array([d,up*su,np.cross(d,up*su)])
        R=Rotation.from_rotvec(p[:3]).as_matrix()@A; t=p[3:6]-R@np.array([0,side*27.85,14.5]);eye=-R.T@t
        if eye[2]>15 and eye[1]*side>0:return R,t,eye
    raise RuntimeError('no upright solution')
def proj(pts,R,t,f=FOCAL):
    q=pts@R.T+t
    return q[...,:2]/q[...,2,None]*f+CENTER
if __name__=='__main__':
    broad = '--broad' in sys.argv
    archived = json.loads((reference/'edges.json').read_text()) if broad else {}
    for n in ([60,195] if broad else seeds):
        if broad:
            pts=[np.array(e) for e in archived[str(n)]['edges']]
        else:
            im=cv.imread(str(raw/f'ring{n}-main.jpg')); es=[edges(im,s) for s in seeds[n]]
            pts=[v[0] for v in es]
        rms,p=fit(pts,48); R,t,eye=world(p,n)
        print(n,rms,eye)
        Path(f'flip-{n}.json').write_text(json.dumps(dict(edges=[x.tolist() for x in pts],R=R.tolist(),t=t.tolist(),f=FOCAL,pp=CENTER.tolist(),roi=[0,0,8160,6120],rms=rms)))

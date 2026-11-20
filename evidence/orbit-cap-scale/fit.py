"""Compare coplanar STL rim projections in full-resolution Flip orbit photographs."""
from pathlib import Path
import json
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

RADII = np.array([13., 9.25])
DZ = 3.75
FOCAL = 662 * 8.16
CENTER = np.array([4080.,3060.])

def transform(p, focal):
    rotation = Rotation.from_rotvec(p[:3]).as_matrix()
    camera = np.array([[focal,0,CENTER[0]],[0,focal,CENTER[1]],[0,0,1.]])
    return camera @ np.column_stack((rotation[:,:2],p[3:6]))

def residual(p, edges, span, focal, radii):
    # The conic's implicit function divided by its image gradient is the
    # first-order normal distance to the predicted rim, in native pixels.
    inv = np.linalg.inv(transform(p,focal))
    gap = np.hypot(p[6] if span is None else span, DZ)
    errors = []
    for uv,offset,r in zip(edges,[0.,gap],radii):
        x,y,z = inv @ uv
        x = x-offset*z
        h = inv[0]-offset*inv[2]
        gradient = 2*(x[:,None]*h[:2]+y[:,None]*inv[1,:2]-r*r*z[:,None]*inv[2,:2])
        errors.extend((x*x+y*y-r*r*z*z)/np.linalg.norm(gradient,axis=1))
    return np.asarray(errors)

def fit(edges,span=None,focal=FOCAL,radii=RADII):
    samples=[np.column_stack((uv,np.ones(len(uv)))).T for uv in edges]
    centers=np.array([(uv.max(axis=0)+uv.min(axis=0))/2 for uv in edges])
    d=centers[1]-centers[0]
    roll=np.arctan2(d[1],d[0])
    z=focal*2*radii[0]/max(np.ptp(edges[0],axis=0))
    starts=[]
    for angle,tilt in [(-45,-12),(45,-12),(-45,12),(45,12),(0,0),(65,0),(-65,0)]:
        r=Rotation.from_euler('xy',[tilt,angle],degrees=True)
        rot=Rotation.from_rotvec([0,0,roll])*r
        p=np.r_[rot.as_rotvec(),(centers[0]-CENTER)*z/focal,z]
        if span is None:p=np.r_[p,45.]
        opt=least_squares(residual,p,args=(samples,span,focal,radii),x_scale='jac',max_nfev=300)
        if span is None:
            # Only the magnitude is observable in the center-distance formula.
            opt.x[6] = abs(opt.x[6])
        starts.append((np.sqrt(np.mean(opt.fun**2)),opt.x))
    return min(starts,key=lambda s:s[0])

if __name__=='__main__':
    for heading,frame in json.loads(Path(__file__).with_name('edges.json').read_text()).items():
        edges=[np.array(p) for p in frame['edges']]
        rms,p=fit(edges)
        print(heading, 'span / free RMS / 42 RMS / 48 RMS',np.round([p[6],rms,fit(edges,42)[0],fit(edges,48)[0]],2))

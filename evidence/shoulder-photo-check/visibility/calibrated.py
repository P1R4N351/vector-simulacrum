"""Check the strongest Flip discriminant with independently calibrated bench camera models."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

def register(camera,edges,side,radii):
    rot=Rotation.from_rotvec(camera['rotation']).as_matrix(); t0=np.array(camera['translation'])
    f=camera['focal_px']*8.16;pp=np.array(camera['principal_point'])*8.16
    K=np.array([[f,0,pp[0]],[0,f,pp[1]],[0,0,1.]])
    ray=(np.r_[(np.mean(edges[0],axis=0)-pp)/f,1])@rot
    eye=-rot.T@t0;front=eye+ray*(14.5-eye[2])/ray[2]
    def residual(v):
        a=np.array([np.cos(v[2]),np.sin(v[2]),0])
        up=np.array([0.,0.,1.]);err=[]
        for i,ed in enumerate(edges):
            center=np.r_[v[:2],14.5]-i*(48*a+3.75*up)
            inv=np.linalg.inv(K@np.column_stack([rot@a,rot@up,rot@center+t0]))
            u,w,h=(np.c_[ed,np.ones(len(ed))]@inv.T).T;r=radii[i]
            grad=2*(u[:,None]*inv[0,:2]+w[:,None]*inv[1,:2]-r*r*h[:,None]*inv[2,:2])
            err.extend((u*u+w*w-r*r*h*h)/np.linalg.norm(grad,axis=1))
        return err
    opts=[least_squares(residual,[*front[:2],ang],loss='soft_l1',f_scale=8) for ang in [0.,1.57,3.14,4.71]]
    opt=min(opts,key=lambda opt:opt.cost);x,y,yaw=opt.x
    obj=Rotation.from_euler('z',yaw).as_matrix()
    origin=np.array([x,y,0])-side*27.85*obj[:,1]
    return rot@obj,rot@origin+t0,f,pp,np.sqrt(np.mean(opt.fun**2))
if __name__=='__main__':
    folder,meshfolder=map(Path,sys.argv[1:3]);models=[np.array(json.loads((meshfolder/file).read_text())).reshape(141,85,3)*10-[2.947435,0,0] for file in ('pre.json','post.json')]
    sys.path.insert(0,str(folder));from scoreflip import outline
    edges=json.loads((folder/'flip-195.json').read_text())['edges'];edges=[np.asarray(v) for v in edges]
    ref=next(a for a in json.loads((folder/'flip-results-broad.json').read_text()) if '195' in a['file'])
    ys=np.array(ref['rows_native_px']);refedge=np.array(ref['observed_edge_x'])
    folder=Path(sys.argv[3])
    rows=[]
    for source,radii in [('cameras',[13,9.25]),('omit-ring180',[13,9.25]),('face-land',[12.75,9])]:
        cam=json.loads((folder/(source+'.json')).read_text())['cameras'][0]
        R,t,f,pp,rms=register(cam,edges,1,radii)
        assert np.allclose(pp,[4080,3060])
        xx=[outline(m,R,t,f,1,ys) for m in models]
        normal=1/np.hypot(1,np.gradient(xx[0],ys))
        rows.append(dict(calibration=source,rim_rms=rms,median_residual=[np.median(abs(x-refedge)*normal) for x in xx],median_registration_shift=np.median(np.abs((xx[0]-ref['old_x'])*normal)), max_signal=np.max(abs((xx[0]-xx[1])*normal))))
    print(json.dumps(rows,indent=2))

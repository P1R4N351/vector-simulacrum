"""Opposite See3CAM orbit views. Input is the calibrated rectified native crop."""
import json, os
from pathlib import Path
import numpy as np
import cv2 as cv
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from scipy.ndimage import map_coordinates, gaussian_filter1d
from scipy.interpolate import PchipInterpolator
P=Path(__file__).parent
source=Path(os.environ.get('VECTOR_CALIBRATION_DIR', next((d/'hub-calibration-b' for d in P.parents if (d/'hub-calibration-b').is_dir()),P/'hub-calibration-b')))
models=Path(os.environ.get('VECTOR_SHELL_EXPORTS', P))
def photo(h):
    archive=Path(os.environ.get('VECTOR_PHOTO_ORBIT', Path.home()/'Pictures/Vector/P2Y8-20261119-three-camera'))
    frame=cv.imread(str(archive/f'ring{h}-see1.jpg'))
    if frame is None or frame.shape[:2]!=(1080,1920):
        raise ValueError('Expected original See3CAM 1920 x 1080 exposure')
    yy,xx=np.indices((780,1200));center=np.array([912.94631722,541.19959849])
    delta=np.stack([xx+310,yy+395],axis=-1)-center
    # Inverse of the shelf-edge radial rectification in see3cam_shelf.py.
    source_px=(center+2*delta/(1+np.sqrt(1+4*1.49896153*np.sum((delta/960)**2,axis=-1,keepdims=True)))).astype('float32')
    return cv.remap(cv.rotate(frame,cv.ROTATE_180),source_px[:,:,0],source_px[:,:,1],cv.INTER_LINEAR)
# Partial arcs of front/rear purple rim, marked in full-resolution rectified crops.
rim={
30:[[[698,548],[693,542],[693,529],[698,513],[706,497],[717,484],[730,474],[739,470],[749,468]],
    [[545,608],[543,602],[544,590],[551,579],[558,570],[566,562],[574,558],[584,556],[591,559],[591,571],[587,584],[580,596],[571,608],[560,613],[551,614]]],
150:[[[455,473],[457,464],[463,459],[471,461],[481,470],[489,482],[498,499],[507,517],[510,528],[507,535],[502,538],[491,535],[481,525],[471,514]],
    [[584,584],[587,575],[595,570],[604,571],[615,581],[625,597],[633,615],[636,630],[634,642],[627,645],[617,640],[608,631],[596,615],[587,597]]]}
# Second pass on contrast-stretched native samples; skip untraceable dark arcs.
first_pass=rim
rim={
30:[[[699,546],[695,541],[694,528],[699,514],[705,503],[710,496],[717,489],[723,485]],[[544,609],[542,602],[541,593],[545,584],[550,576],[555,570],[564,563],[572,559],[582,556],[553,613],[559,614]]],
150:[[[454,473],[457,464],[463,458],[471,459],[484,467],[493,479],[502,497],[511,516],[513,527],[510,535],[502,541],[490,538],[480,527],[467,513]],[[583,584],[585,574],[593,568],[605,569],[618,580],[628,595],[636,613],[638,631],[634,645],[626,648],[613,642],[604,633],[594,614],[585,597]]]}
class Cam:
 def __init__(self,cam):
    self.f=cam['focal_px'];self.pp=np.array(cam['principal_point']);self.rot=Rotation.from_rotvec(cam['rotation']).as_matrix();self.t=np.array(cam['translation']);self.eye=-self.rot.T@self.t
 def pr(self,p):
    q=p@self.rot.T+self.t;return self.pp+self.f*q[:,:2]/q[:,2,None]
 def on_plane(self,p,z):
    r=self.rot.T@np.r_[(p-self.pp)/self.f,1];return self.eye+r*((z-self.eye[2])/r[2])
def rz(theta):
    s,c=np.sin(theta),np.cos(theta)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
def solve(cam,picks,diam=(26,18.5),arcs=None):
    picks=[np.array(p,float) for p in picks];up=np.array([0.,0.,1.]);K=np.array([[cam.f,0,cam.pp[0]],[0,cam.f,cam.pp[1]],[0,0,1]])
    centers=[cam.on_plane(np.mean(p,axis=0),z) for p,z in zip(picks,[14.5,10.75])]
    dir=centers[0]-centers[1];guess=np.r_[centers[0][:2],np.arctan2(dir[1],dir[0])]
    def res(v):
        x,y,a=v;axis=rz(a)[:,0];err=[]
        for i,rad in enumerate(np.array(diam)/2):
            cen=[x,y,14.5]-i*(48*axis+3.75*up)
            inv=np.linalg.inv(K@np.column_stack([cam.rot@axis,cam.rot@up,cam.rot@cen+cam.t]))
            pix=picks[i] if arcs is None else picks[i][arcs[i]]
            u,vv,w=(np.c_[pix,np.ones(len(pix))]@inv.T).T
            grad=2*(u[:,None]*inv[0,:2]+vv[:,None]*inv[1,:2]-rad**2*w[:,None]*inv[2,:2])
            err.extend((u*u+vv*vv-rad*rad*w*w)/np.linalg.norm(grad,axis=1))
        return err
    fit=least_squares(res,guess,loss='soft_l1',f_scale=2.)
    x,y,a=fit.x;R=rz(a);pos=np.array([x,y,0]);s=np.sign((cam.eye-pos)@R[:,1]);origin=pos-s*27.84919*R[:,1]
    return cam.rot@R,cam.rot@origin+cam.t,s,np.sqrt(np.mean(np.asarray(res(fit.x))**2))
def proj(p,rt,cam):
    r,t,*_=rt;q=p@r.T+t;return cam.pp+cam.f*q[:,:2]/q[:,2,None]
def depth_read(name):
    return np.array(json.loads((models/name).read_text())).reshape(-1,3)*10-[2.947435,0,0]
M=[depth_read('pre.json'),depth_read('post.json')]
def camera(name):return Cam(json.loads((source/(name+'.json')).read_text())['cameras'][2])
def calibrate():
    cam=camera('cameras');out={}
    for h in rim:
        rt=solve(cam,rim[h]);R,t,s,rms=rt
        out[str(h)]={'R':R.tolist(),'t':t.tolist(),'focal_px':cam.f,'principal_point':cam.pp.tolist(),'rim_side':s,'rim_rms_px':rms,'note':'robot mm x forward, y left, z up, origin under front axle; world_to_optical=R*robot+t; rectified 1200x780 crop'}
    (P/'poses.json').write_text(json.dumps(out,indent=2))
if __name__=='__main__':calibrate()

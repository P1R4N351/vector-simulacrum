"""Fit dock-registered cameras to B-only wheel rims with known radii and axle spacing."""
from pathlib import Path
import argparse
import json
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent
src=json.loads((HERE/'inputs.json').read_text());obs=json.loads((HERE/'edges.json').read_text()); tags=list(obs)
P=np.asarray(src['principals']);dock=np.asarray(src['world']);measured=np.asarray(src['glyph_lip'])
seed=np.r_[np.ravel(src['camera_seeds']),np.ravel(src['pose_seeds'])]
def calc(p,omit=None,face=False,report=False):
 cameras=p[:21].reshape(3,7);poses=p[21:].reshape(3,3)
 fs=cameras[:,6]
 residuals=[];diagnostics={}
 for i,cam in enumerate(cameras):
  rot=Rotation.from_rotvec(cam[:3]).as_matrix()
  K=np.array([[fs[i],0,P[i,0]],[0,fs[i],P[i,1]],[0,0,1]])
  t=cam[3:6]
  proj=(dock@rot.T+t)@K.T
  diff=proj[:,:2]/proj[:,2:]-measured[i]
  residuals.extend((diff/np.array([1.,1.,1.,1.,3.])[:,None]).ravel())
  diagnostics['dock'+str(i)]=diff.tolist()
  for j,(tag,pose) in enumerate(zip(tags,poses)):
   x,y,a=pose
   # Horizontal front-to-rear spacing, vertical front-to-rear drop.
   ax=np.array([np.cos(a),np.sin(a),0.]);up=np.array([0.,0.,1.])
   diagnostics[tag+str(i)]=[]
   for wheel,radius in enumerate([12.75,9.] if face else [13.,9.25]):
    center=np.array([x,y,14.5])-wheel*(48*ax+3.75*up)
    H=K@np.column_stack((rot@ax,rot@up,rot@center+t))
    inv=np.linalg.inv(H);uv=np.asarray(obs[tag][i][wheel])
    u,v,w=inv@np.column_stack((uv,np.ones(len(uv)))).T
    grad=2*(u[:,None]*inv[0,:2]+v[:,None]*inv[1,:2]-radius**2*w[:,None]*inv[2,:2])
    delta=(u*u+v*v-radius**2*w*w)/np.linalg.norm(grad,axis=1)
    diagnostics[tag+str(i)].append(float(np.sqrt(np.mean(delta**2))))
    # Decimate correlated samples; use fixed indices at each evaluation.
    if tag!=omit:residuals.extend(delta[::3]/[1.1,1.2,2.0][i])
 # Rims and dock positions carry metric scale. Focal and phone baseline priors stabilize depth.
 residuals.extend((fs-[662,376,580])/[32,19,150])
 centers=[Rotation.from_rotvec(-cam[:3]).apply(-cam[3:6]) for cam in cameras]
 residuals.append((np.linalg.norm(centers[0]-centers[1])-14.232)/2)
 return diagnostics if report else np.asarray(residuals)
def center_res(p, omit=None):
 cameras=p[:21].reshape(3,7);poses=p[21:].reshape(3,3)
 out=[]
 for i,cam in enumerate(cameras):
  rot=Rotation.from_rotvec(cam[:3])
  def project(points):
   v=rot.apply(points)+cam[3:6]
   return v[:,:2]/v[:,2,None]*cam[6]+P[i]
  out.extend(((project(dock)-measured[i])/np.array([1,1,1,1,3])[:,None]).flatten())
  for j,tag in enumerate(tags):
   if tag == omit:
    continue
   x,y,a=poses[j];xyz=np.array([[x,y,14.5],[x-48*np.cos(a),y-48*np.sin(a),10.75]])
   centers=[(np.min(v,axis=0)+np.max(v,axis=0))/2 for v in obs[tag][i]]
   out.extend(((project(xyz)-centers)/[1.5,1.5,2.5][i]).flatten())
 out.extend((cameras[:,6]-[662,376,580])/[32,19,150])
 return np.asarray(out)

def fit(name='cameras',omit=None,face=False):
 start=least_squares(center_res,seed,args=(omit,),x_scale='jac').x
 if name!='cameras' and omit is None: start=np.asarray(json.loads((HERE/'cameras.json').read_text())['parameters'])
 start=least_squares(calc,start,args=(omit,face),x_scale='jac',max_nfev=90).x
 res=least_squares(calc,start,args=(omit,face),loss='soft_l1',x_scale='jac',max_nfev=170)
 if omit is not None:
  # Register the omitted robot pose afterward with the fitted cameras fixed.
  j=tags.index(omit)
  sl=slice(21+3*j,24+3*j)
  def register_robot_pose(x):
   trial=res.x.copy()
   trial[sl]=x
   return calc(trial,None,face)
  res.x[sl]=least_squares(register_robot_pose,seed[sl],loss='soft_l1',x_scale='jac').x
 cams=[dict(rotation=p[:3].tolist(),translation=p[3:6].tolist(),focal_px=p[6],principal_point=P[i].tolist()) for i,p in enumerate(res.x[:21].reshape(3,7))]
 diagnostics=calc(res.x,omit,face,True)
 out=dict(cameras=cams,parameters=res.x.tolist(),B_rms=diagnostics,solver_success=bool(res.success),solver_message=res.message)
 (HERE/(name+'.json')).write_text(json.dumps(out,indent=2)+'\n')
 print(name,res.cost,res.nfev,[(k,np.round(v,2)) for k,v in diagnostics.items() if not k.startswith('dock')])
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['cameras', 'omit-ring180', 'face-land'], default='cameras')
    variant = parser.parse_args().variant
    fit(variant, omit='ring180' if variant == 'omit-ring180' else None, face=variant == 'face-land')

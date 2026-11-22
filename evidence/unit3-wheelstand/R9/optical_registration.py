"""Compare adjacent-face cube poses from two docked shelf views, in dock coordinates."""
import csv
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[2]))
from see3cam_shelf import WORLD,image_points,locate,edge_groups,lens_fit

# Manually picked corners on the half-resolution recordings (clockwise, far left first).
glyph=np.array([[405,277.5],[439.5,277.5],[439.5,311],[405,311],[420.5,363.]])*2
corners=np.array([[[170,187],[330,157],[361,242],[184,269]],
                  [[183,233],[364,213],[377,304],[191,319]]],float)/8+[282,295]
lens=lens_fit(edge_groups())
def fit_camera(picks, first=None):
    def residual(p):
        pix=(image_points(WORLD,p,lens)-picks)/1.6
        residuals=np.r_[pix.ravel(),(p[6]-473.11)/100]
        if first is not None:
            pts=locate(first*2,44,p,lens)
            lengths=np.linalg.norm(np.roll(pts,-1,axis=0)-pts,axis=1)
            diagonals=np.linalg.norm(pts[:2]-pts[2:],axis=1)
            residuals=np.r_[residuals,(lengths-23)/2,(diagonals-23*np.sqrt(2))/2.8]
        return residuals
    return least_squares(residual,[1.8,-.08,.12,-35,71,389,473.11],
                         bounds=([-7]*3+[-2000,-2000,50,250],[7]*3+[2000,2000,2000,1400])).x

sdk=np.array([[26.481335,-103.98442,-1.01269846],[41.602207,-95.55888,-.0]])
ob=next(r for r in csv.DictReader((here/'roll-poses.csv').open()) if r['source']=='robot_observed_object' and float(r['sec'])<0)
sdk[1,2]=float(ob['yaw_rad'])
def measure(p, uv, height):
    result=[]
    for j,q in enumerate(uv):
        pts=locate(q*2,height,p,lens)
        right=(pts[1]-pts[0]+pts[2]-pts[3])/2
        deep=(pts[0]-pts[3]+pts[1]-pts[2])/2
        angle=np.rad2deg(np.arctan2(right[1],right[0]))
        diff=(angle-np.degrees(sdk[j,2])+45)%90-45
        result.append([*pts.mean(axis=0),np.linalg.norm(right),np.linalg.norm(deep),angle,diff])
    return np.array(result)
rng=np.random.default_rng(147)
baseline=measure(fit_camera(glyph),corners,44)
r=measure(fit_camera(glyph,corners[0]),corners,44)
samples=[]
for _ in range(140):
    drawn=corners+rng.uniform(-1,1,corners.shape)
    p=fit_camera(glyph+rng.uniform(-2,2,glyph.shape),drawn[0])
    samples.append(measure(p,drawn,44+rng.uniform(-1,1)))
samples=np.array(samples)
# Dock-forward points toward the camera (-world Y); dock-left points to image right (+world X).
delta=np.diff(r[:,:2],axis=0)[0]
opt=np.array([-delta[1],delta[0]])
ds=sdk[1,:2]-sdk[0,:2]
print('lens',lens,'pose',fit_camera(glyph),'per view',r,'optical vs sdk change',opt,ds)
with (here/'optical-registration.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n');w.writerow(('calibration','view','world_x_mm','world_y_mm','marker_width_mm','marker_depth_mm','edge_yaw_deg','grid_yaw_minus_sdk_deg'))
    for method,values in [('Dock only',baseline),('Dock and first cube',r)]:
        w.writerows([[method,label,*row] for label,row in zip(('Before roll','Next approach'),values)])
fig,axes=plt.subplots(1,2,figsize=(9,4),layout='constrained')
for i,label in enumerate(('Forward','Left')):
    change=np.diff(samples[:,:,:2],axis=1)[:,0,:]
    vals=-change[:,1] if i==0 else change[:,0]
    low,high=np.percentile(vals,[5,95])
    print(label,low,high)
    axes[0].plot([i,i],[low,high],c='C0')
axes[0].scatter([0,1],opt,label='Overhead')
axes[0].scatter([0,1],ds,marker='x',label='SDK')
axes[0].set(xticks=[0,1],xticklabels=['Dock-forward','Dock-left'],ylabel='Change in cube center (mm)')
axes[0].legend()
for j,run in enumerate(('Before roll','Next approach')):
    vals=samples[:,j,-1];low,high=np.percentile(vals,[5,95]);print(run,'yaw',low,high)
    axes[1].plot([j,j],[low,high],c='C0')
axes[1].scatter([0,1],r[:,-1]);axes[1].axhline(0,c='.5',lw=.8)
axes[1].set(xticks=[0,1],xticklabels=['Before roll','Next approach'],ylabel='Optical grid minus SDK yaw (°)')
fig.savefig(here/'optical-registration.png',dpi=150)
fig,axes=plt.subplots(1,3,figsize=(10,3.5),layout='constrained')
for ax,filename,points,offset,title in zip(axes,['dock-glyph.png','docked-r8.png','docked-r9.png'],
        [glyph[:4]/2,corners[0],corners[1]],[[360,260],[260,265],[260,265]],
        ['Dock calibration','Before successful roll','Held-out approach']):
    ax.imshow(plt.imread(here/filename),interpolation='nearest')
    poly=points[[0,1,2,3,0]]-offset
    ax.plot(*poly.T,c='#ff724c',lw=.8)
    if filename=='dock-glyph.png':ax.scatter(*(glyph[4]/2-offset),s=14,c='#ff724c')
    ax.set_title(title);ax.axis('off')
fig.savefig(here/'registration-picks.png',dpi=150)

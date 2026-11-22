"""Measure chassis translation relative to the stationary cube before contact."""
import csv
import sys
from pathlib import Path
import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from approach_frames import frames
from pickup_onset import read

here=Path(__file__).resolve().parent
t,K,images=frames(here)
gray=[cv2.cvtColor(im,cv2.COLOR_BGR2GRAY) for im in images]
unit=np.array([[-.5,-.5,0],[.5,-.5,0],[.5,.5,0],[-.5,.5,0]])
corners=np.loadtxt(here/"initial-glyph-corners.csv",delimiter=",",skiprows=1)
def face_basis(p):
    ok,rv,_=cv2.solvePnP(unit,p,K,None)
    assert ok
    return cv2.Rodrigues(rv)[0]

def obliquity(dx,dy):
    return (np.rad2deg(np.arctan2(dy,dx))+45)%90-45

def optical_angle(basis, pixels):
    rays=np.column_stack((pixels,np.ones(len(pixels))))@np.linalg.inv(K).T
    # The glyph normal points inward. For a fixed-height landmark the unknown
    # plane offset multiplies both coordinates by the same constant.
    xy=(rays/(rays@basis[:,2])[:,None])@basis[:,:2]
    delta=xy[-1]-xy[0]
    return obliquity(delta[0],-delta[1])

s=read(here/"sightings.csv")
base=s[s["sec"]<0][-1]
cube_yaw=Rotation.from_quat([base[k] for k in ("qx","qy","qz","qw")]).as_euler("xyz")[2]
a=read(here/"approach.csv")
def sdk_angle(times):
    d=np.array([np.diff(np.interp(times,a["sec"],a[k])).item() for k in ("x","y")])
    c,s=np.cos(cube_yaw),np.sin(cube_yaw)
    return obliquity(c*d[0]+s*d[1],-s*d[0]+c*d[1])

# The robot turned during approach. Compare the leading patch with the path
# of each possible fork vertex, independent of which point the camera tracked.
sys.path.insert(0,str(here.parents[1]/"unit3-cube-accelerometer"))
from fork_envelope import registered_fork
from simulacrum import params
vertices=np.unique(registered_fork(Path(sys.argv[1])).reshape(-1,3),axis=0)
robot=params.values(params.load(model="vector_2"))
def fork_xy(time):
    x,y,yaw,pitch,h=[np.interp(time,a["sec"],a[k]) for k in ("x","y","yaw","pitch","lift")]
    bx,_,bz=robot["lift_base_position_mm"]
    reach=bx+np.sqrt(robot["lift_arm_length_mm"]**2-(h-bz)**2)
    z0=robot["wheel_front_radius_mm"]
    f=(vertices[:,0]+reach)*np.cos(pitch)-(vertices[:,2]+h-z0)*np.sin(pitch)
    return np.array([x+f*np.cos(yaw)-vertices[:,1]*np.sin(yaw),
                     y+f*np.sin(yaw)+vertices[:,1]*np.cos(yaw)])
def fork_angles(first,last):
    dx,dy=fork_xy(last)-fork_xy(first)
    c,s=np.cos(cube_yaw),np.sin(cube_yaw)
    angles=obliquity(c*dx+s*dy,-s*dx+c*dy)
    return angles.min(),angles.max()

patches=[("Rear pivot area",(378,123,412,156)),("Fork front",(232,142,263,182)),
         ("Head side",(321,113,351,148))]
ids=np.arange(65,78)
rng=np.random.default_rng(65)
bases=[face_basis(corners+rng.uniform(-1,1,(4,2))) for _ in range(500)]
ref=face_basis(corners)
fig,ax=plt.subplots(figsize=(7,4),layout="constrained")
rows=[]
tiles=[images[i].copy() for i in (65,71,77)]
for name,(x,y,u,v) in patches:
    template=gray[65][y:v,x:u]
    xy=[];quality=[]
    for i in ids:
        search=gray[i][max(y-18,0):min(v+24,240),x-135:min(420,u+8)]
        match=cv2.matchTemplate(search,template,cv2.TM_CCOEFF_NORMED)
        _,score,_,(xx,yy)=cv2.minMaxLoc(match)
        left,top=x-135+xx,max(y-18,0)+yy
        xy.append([left+(u-x)/2,top+(v-y)/2]);quality.append(score)
        if i in (65,71,77):
            tile=tiles[(65,71,77).index(i)]
            cv2.rectangle(tile,(left,top),(left+u-x,top+v-y),(0,220,255),1)
    xy=np.array(xy)
    for end in (71,74,77):
        j=end-65
        pixels=xy[[0,j]]
        ang=optical_angle(ref,pixels)
        perturbed=[optical_angle(b,pixels+rng.uniform(-1,1,(2,2))) for b in bases]
        lo,hi=np.percentile(perturbed,[5,95])
        rows.append([name,65,end,t[65],t[end],min(quality[:j+1]),ang,lo,hi,sdk_angle(t[[65,end]]),*fork_angles(t[65],t[end])])
    ax.plot(t[[71,74,77]], [r[6] for r in rows[-3:]],"o-",label=name)
for tile,i in zip(tiles,(65,71,77)):
    cv2.putText(tile,f"{t[i]:.2f} s",(5,22),0,.55,(255,255,255),1,cv2.LINE_AA)
cv2.imwrite(str(here/"approach-landmarks.jpg"),np.vstack(tiles))
ax.plot(t[[71,74,77]],[r[9] for r in rows[-3:]],"ks--",label="SDK planar path")
ax.fill_between(t[[71,74,77]],[r[10] for r in rows[-3:]],[r[11] for r in rows[-3:]],alpha=.2,label="SDK path · all fork vertices")
ax.set(xlabel="End time of interval starting at frame 65 (s from command)",ylabel="Travel angle from nearest cube axis (°)")
ax.legend()
fig.savefig(here/"approach-direction.png",dpi=160)
with (here/"approach-direction.csv").open("w") as stream:
    w=csv.writer(stream,lineterminator="\n")
    w.writerow("patch start_frame end_frame start_s end_s min_corr optical_deg jitter_p05 jitter_p95 sdk_deg fork_min_deg fork_max_deg".split())
    w.writerows(rows)

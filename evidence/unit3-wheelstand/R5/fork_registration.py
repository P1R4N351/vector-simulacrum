"""Independent projection of the measured approach onto the fixed shelf camera."""
import csv
import sys
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from approach_frames import frames
from pickup_onset import read
here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[1]/"unit3-cube-accelerometer"))
from fork_envelope import registered_fork
from simulacrum import params

t,K,imgs=frames(here)
tri=registered_fork(Path(sys.argv[1]))
verts,ids=np.unique(tri.reshape(-1,3),axis=0,return_inverse=True)
faces=ids.reshape(-1,3)
spec=params.values(params.load(model="vector_2"))
raw=read(here/"sightings.csv")
s=raw[raw["sec"]<0][-1]
heading=Rotation.from_quat([s[k] for k in ("qx","qy","qz","qw")]).as_euler("xyz")[2]
a=read(here/"approach.csv")
unit=np.array([[-.5,-.5,0],[.5,-.5,0],[.5,.5,0],[-.5,.5,0]])
corners=np.loadtxt(here/"initial-glyph-corners.csv",delimiter=",",skiprows=1)
def sdk_points(i):
    x,y,yaw,p,h=[np.interp(t[i],a["sec"],a[k]) for k in ("x","y","yaw","pitch","lift")]
    bx,_,bz=spec["lift_base_position_mm"]
    reach=bx+np.sqrt(spec["lift_arm_length_mm"]**2-(h-bz)**2)
    z0=spec["wheel_front_radius_mm"]
    pts=Rotation.from_euler("y",-p).apply(verts+[reach,0,h-z0])+[0,0,z0]
    pts=Rotation.from_euler("z",yaw).apply(pts)+[x-s["x"],y-s["y"],0]
    return Rotation.from_euler("z",-heading).apply(pts)

def camera_pose(corners):
    ok,rv,tv=cv2.solvePnP(unit,corners,K,None)
    assert ok
    return cv2.Rodrigues(rv)[0], tv.ravel()

def project(pts,pose,width):
    # SDK +y is the approach face. In frame 65 the robot is right of the
    # square, identifying +y with the glyph's image-right edge.
    glyph=np.column_stack((pts[:,1],pts[:,0],44-pts[:,2]))
    r,v=pose
    cam=glyph@r.T+width*v
    assert (cam[:,2]>0).all()
    xy=cam@K.T
    return xy[:,:2]/xy[:,2:]

def outline(pixels):
    mask=np.zeros((240,420),np.uint8)
    for f in np.rint(pixels[faces]).astype(np.int32):
        cv2.fillConvexPoly(mask,f,255)
    return cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]

pose=camera_pose(corners)
# Scan the straight leading edge above the small forward projection, away from
# the bright cube. Brackets were selected on the unoverlaid rectified photos.
def measured_edge(image,left):
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY).astype(float)
    profile=gray[144:149,left:left+23].mean(axis=0)
    dark=profile[-4:].mean()
    light=profile[:4].mean()
    assert light-dark>9
    edges=[]
    for fraction in (.35,.5,.65):
        threshold=light+fraction*(dark-light)
        i=np.flatnonzero((profile[:-1]>=threshold)&(profile[1:]<threshold))[0]
        edges.append(left+i+(profile[i]-threshold)/(profile[i]-profile[i+1]))
    return edges

def silhouette_x(pixels):
    # Intersect triangle edges with one row; leftmost intersection is the
    # model boundary even where one triangle occludes another.
    p=pixels[faces]
    q=np.roll(p,1,axis=1)
    result=[]
    for y in range(144,149):
        keep=(p[:,:,1]>y)!=(q[:,:,1]>y)
        a,b=p[keep],q[keep]
        if len(a):
            result.append(np.min(a[:,0]+(y-a[:,1])*(b[:,0]-a[:,0])/(b[:,1]-a[:,1])))
    return np.mean(result) if len(result)==5 else np.nan

tiles=[]
rows=[]
rng=np.random.default_rng(77)
poses=[camera_pose(corners+rng.uniform(-1,1,(4,2))) for _ in range(80)]
for i,left in ((65,228),(71,202),(77,175)):
    img=imgs[i].copy()
    edge=measured_edge(img,left)
    pts=sdk_points(i)
    for width,color in [(20,(220,110,0)),(23,(0,210,255))]:
        pix=project(pts,pose,width)
        cv2.drawContours(img,outline(pix),-1,color,1,cv2.LINE_AA)
        screen=silhouette_x(pix)
        varied=np.array([silhouette_x(project(pts,r,width)) for r in poses])
        valid=varied[np.isfinite(varied)]
        rows.append([i,t[i],width,*edge,screen,screen-edge[1],len(valid),
                     *np.percentile(valid-edge[1],[5,95])])
    cv2.line(img,(round(edge[1]),143),(round(edge[1]),149),(255,0,255),2)
    tile=img[60:225,70:360].copy()
    cv2.putText(tile,f"frame {i}   {t[i]:+.2f} s",(6,17),0,.5,(255,255,255),1,cv2.LINE_AA)
    tiles.append(tile)
sheet=np.hstack(tiles)
sheet=cv2.copyMakeBorder(sheet,0,28,0,0,cv2.BORDER_CONSTANT)
for x,label,color in [(8,"Observed edge",(255,0,255)),(185,"20 mm glyph",(220,110,0)),(370,"23 mm glyph",(0,210,255))]:
    cv2.putText(sheet,label,(x,sheet.shape[0]-8),0,.5,color,1,cv2.LINE_AA)
cv2.imwrite(str(here/"fork-registration.jpg"),sheet)
with (here/"fork-registration.csv").open("w") as stream:
    w=csv.writer(stream,lineterminator="\n")
    w.writerow("frame sec glyph_mm edge35_px edge50_px edge65_px model_px residual_px valid_jitter_draws residual_p05_px residual_p95_px".split())
    w.writerows(rows)

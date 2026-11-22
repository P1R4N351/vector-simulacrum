"""Check a camera-pitch hypothesis on the stationary scene, independent of the cube."""
import argparse
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import reprojection as rp

here = Path(__file__).resolve().parent
tracks = here / "retreat-background.csv"
K = rp.models["Unit 3"]
inv = np.linalg.inv(K)
_, first, last = rp.pairs[-1]
a, A = rp.camera_pose(first)
b, B = rp.camera_pose(last)
initial_pos = (a-b) @ B
initial_rot = A.T @ B


def extract(folder):
    times = [1795287726.9763,1795287727.3533,1795287727.8700]
    ends = [1795287739.6582,1795287740.0327,1795287740.6476]
    rows = []
    for pair, (t0,t1) in enumerate(zip(times,ends)):
        images = [cv2.imread(str(next(folder.glob(f"{t:.4f}-*.jpg"))),0) for t in (t0,t1)]
        mask = np.zeros_like(images[0])
        # The cube occupies the middle. Include left shelving, right background and sphere.
        mask[5:275,5:177] = 255
        mask[5:270,495:780] = 255
        pts = cv2.goodFeaturesToTrack(images[0],220,0.017,10,mask=mask)
        opts = dict(winSize=(21,21),maxLevel=3)
        nxt,ok,_ = cv2.calcOpticalFlowPyrLK(*images,pts,None,**opts)
        back,rev,_ = cv2.calcOpticalFlowPyrLK(*images[::-1],nxt,None,**opts)
        keep = ok.ravel().astype(bool) & rev.ravel().astype(bool)
        keep &= np.linalg.norm(pts-back,axis=2).ravel()<0.5
        for p,q in zip(pts[keep,0],nxt[keep,0]):
            rows.append((pair,t0,t1,*p,*q))
    with tracks.open("w") as f:
        w = csv.writer(f,lineterminator="\n")
        w.writerow(["pair","start","end","u0","v0","u1","v1"])
        w.writerows(rows)


def distance(pitch, points, position=initial_pos):
    # Each initial pixel sweeps an epipolar line over possible scene depths.
    # Negative rotation here moves a central feature down in the final image.
    change = Rotation.from_euler("x",float(pitch[0]),degrees=True).as_matrix()
    rays = np.column_stack((points[:,0:2],np.ones(len(points)))) @ inv.T
    lines = np.cross(position @ change.T, rays @ initial_rot @ change.T) @ inv
    final = np.column_stack((points[:,2:4],np.ones(len(points))))
    return np.sum(lines*final,axis=1) / np.linalg.norm(lines[:,:2],axis=1)


def fit(points):
    return least_squares(distance,[0.],args=(points,),bounds=(-3,3),loss="soft_l1",f_scale=.5).x[0]


parser = argparse.ArgumentParser()
parser.add_argument("--extract",type=Path)
args = parser.parse_args()
if args.extract:
    extract(args.extract)
data = np.loadtxt(tracks,delimiter=",",skiprows=1)
cube = np.column_stack((first["quad"],last["quad"]))
cube_pitch = fit(cube)
fig, axes = plt.subplots(1,2,figsize=(9,3.6),layout="constrained")
angles = np.linspace(-1.3,1.3,131)
for side, ax in zip(["Left","Right"],axes):
    for i in range(3):
        p = data[data[:,0]==i,3:]
        p = p[p[:,0]<200] if side=="Left" else p[p[:,0]>490]
        pitch = fit(p)
        residual = lambda v: np.median(np.abs(distance([v],p)))
        print(side,i,len(p),round(pitch,3),round(residual(0),2),round(residual(cube_pitch),2))
        ax.plot(angles,[residual(v) for v in angles],label=f"Frame pair {i+1}")
    ax.axvline(cube_pitch,color="k",ls="--",label="Cube-only fit")
    ax.set(title=f"{side} of cube",xlabel="Extra camera x rotation (degrees)",ylabel="Median epipolar error (px)")
axes[0].legend()
print("cube-only fit",cube_pitch)
fig.savefig(here/"retreat-background.png",dpi=160)

# A vertical camera translation changes near-object flow more than distant flow.
# Test a single vertical offset without altering the frozen head extrinsics.
base = initial_pos.copy()
vertical = np.array([0.,0.,.001]) @ B
def elevation(mm, points):
    return distance([0.],points,base - mm[0]*vertical)
for name,p in [("cube",cube),("scene",data[:,3:]),("left",data[data[:,3]<200,3:]),("right",data[data[:,3]>490,3:])]:
    offset = least_squares(elevation,[0.],args=(p,),loss="soft_l1",f_scale=.5,bounds=(-6,6)).x
    print(name,"vertical mm",offset, "median px scene/cube",*[round(np.median(np.abs(elevation(offset,s))),2) for s in (data[:,3:],cube)])

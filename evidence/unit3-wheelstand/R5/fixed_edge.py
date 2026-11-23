"""Project one-DOF rotations around each candidate lower edge into the R5 camera."""
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares, minimize_scalar
from scipy.spatial.transform import Rotation

here = Path(__file__).resolve().parent
glyph = np.genfromtxt(here/"glyph-raise.csv", delimiter=",", names=True)
k = np.array([[340.,0,209.5],[0,340.,119.5],[0,0,1]])
unit = np.array([[-.5,-.5,0],[.5,-.5,0],[.5,.5,0],[-.5,.5,0]])
picks = np.array([[[r[f"x{i}"],r[f"y{i}"]] for i in range(1,5)] for r in glyph])
edges = [(0,-1),(1,1),(0,1),(1,-1)]  # far, right, near, left in the starting glyph image


def fits(start, end, width, slide=False):
    square = unit*width
    ok, r, t = cv2.solvePnP(square,start,k,None,flags=cv2.SOLVEPNP_ITERATIVE)
    assert ok and t[2,0]>0
    base = cv2.Rodrigues(r)[0]
    result = []
    for axis, side in edges:
        pivot = np.array([0.,0.,44.])
        pivot[1-axis] = side*22
        direction = np.eye(3)[axis]
        def pixel_errors(angle, xy):
            points = Rotation.from_rotvec(direction*angle).apply(square-pivot)+pivot
            points[:, :2] += xy
            camera = points @ base.T+t.ravel()
            pixel = camera @ k.T
            return pixel[:,:2]/pixel[:,2:] - end
        def residual(angle):
            return np.sqrt(np.mean(np.sum(pixel_errors(angle, [0.,0.])**2, axis=1)))
        # Only tilt directions that lift the opposite edge are physically available.
        sign = side if axis==0 else -side
        opt = minimize_scalar(lambda a: residual(sign*a),bounds=(0,np.pi/3),method="bounded")
        if slide:
            sol = least_squares(lambda v: pixel_errors(sign*v[0],v[1:]).ravel(),
                                [opt.x,0.,0.],bounds=([0,-np.inf,-np.inf],[np.pi/3,np.inf,np.inf]))
            result.append((np.degrees(sol.x[0]),np.sqrt(np.sum(sol.fun**2)/4),*sol.x[1:]))
        else:
            result.append((np.degrees(opt.x),opt.fun))
    return np.array(result)


labels = ["Far edge","Right edge","Near edge","Left edge"]
fig, ax = plt.subplots(figsize=(8,4),layout="constrained")
rng = np.random.default_rng(94)
with (here/"fixed-edge.csv").open("w") as f:
    out=csv.writer(f)
    out.writerow("frame wrist_mm model angle_deg rms_px rms_p05_px rms_p95_px best_share width20_rms_px".split())
    for g, target in zip(glyph[1:4], picks[1:4]):
        fit = fits(picks[0],target,23)
        small = fits(picks[0],target,20)
        noise = np.array([fits(picks[0]+rng.uniform(-1,1,(4,2)),target+rng.uniform(-1,1,(4,2)),23)
                          for _ in range(200)])
        for i,label in enumerate(labels):
            low,high = np.percentile(noise[:,i,1],[5,95])
            share = np.mean(np.argmin(noise[:,:,1],axis=1)==i)
            out.writerow([int(g["frame"]),g["wrist_mm"],label,*fit[i],low,high,share,small[i,1]])
            x=g["wrist_mm"]+(i-1.5)*.32
            ax.plot(x,fit[i,1],"o",color=f"C{i}",label=label if g["frame"]==89 else None)
            ax.vlines(x,low,high,color=f"C{i}")
        immobile=np.sqrt(np.mean(np.sum((target-picks[0])**2,axis=1)))
        ax.plot(g["wrist_mm"],immobile,"kx",label="Unmoved glyph" if g["frame"]==89 else None)
        out.writerow([int(g["frame"]),g["wrist_mm"],"Unmoved",0,immobile,"","","",""])
ax.set(xlabel="Recorded wrist height (mm)",ylabel="Corner reprojection error (RMS pixels)")
ax.legend(fontsize=8,ncol=3)
fig.savefig(here/"fixed-edge.png",dpi=160)
with (here/"sliding-edge.csv").open("w") as stream:
    writer=csv.writer(stream)
    writer.writerow(["frame","edge","tilt_deg","rms_px","slide_x_mm","slide_y_mm","heldout_light_rms_px"])
    lights = np.genfromtxt(here/"raise-lights.csv",delimiter=",",names=True)
    def lamp_xy(row):
        return np.float32([[row["red_x_px"],row["red_y_px"]],[row["cyan_x_px"],row["cyan_y_px"]]])
    initial = lamp_xy(lights[0])
    ok,r,t = cv2.solvePnP(unit*23,picks[0],k,None)
    assert ok
    base = cv2.Rodrigues(r)[0]
    for g,target in zip(glyph[1:4],picks[1:4]):
        for label,(axis,side),fit in zip(labels,edges,fits(picks[0],target,23,slide=True)):
            pivot=np.array([0.,0.,44.])
            pivot[1-axis]=22*side
            turn=np.radians(fit[0])*(side if axis==0 else -side)
            points=Rotation.from_rotvec(np.eye(3)[axis]*turn).apply(unit*23-pivot)+pivot
            points[:,:2]+=fit[2:]
            camera=points @ base.T+t.ravel()
            projected=camera @ k.T
            dest=projected[:,:2]/projected[:,2:]
            h=cv2.getPerspectiveTransform(picks[0].astype("float32"),dest.astype("float32"))
            predicted=cv2.perspectiveTransform(initial[None],h)[0]
            measured=lamp_xy(lights[lights["frame"]==g["frame"]][0])
            error=np.sqrt(np.mean(np.sum((predicted-measured)**2,axis=1)))
            writer.writerow([int(g["frame"]),label,*fit,error])

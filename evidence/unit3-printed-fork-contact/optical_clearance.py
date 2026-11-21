"""Register trial 1's marker poses to the replay, with and without shelf support."""
import csv
import cv2
import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from clearance import closest
from evaluate import run, here
from tilt import square, p

# Glyph coordinates to initial cube coordinates: glyph right = cube left,
# glyph down = cube forward, glyph positive normal = cube down.
G = np.array([[0,1,0],[1,0,0],[0,0,-1]])
marker = square @ G.T + [0,0,22]
rows = p[p["trial"] == 1]
quads = np.array([list(r)[2:] for r in rows]).reshape(-1,4,2)


def reconstruct(k, focal, on_shelf, corners=quads, times=rows["sec"]):
    c = np.array([912.9463,541.1996])
    K = np.array([[focal,0,c[0]],[0,focal,c[1]],[0,0,1.]])
    def pixels(q):
        d = q + [420,485] - c
        return c + d / (1 + k*np.sum((d/960)**2,axis=1)[:,None])
    def solve(uv):
        ok, r, t = cv2.solvePnP(marker,uv,K,None)
        if not ok:
            raise ValueError("Cube pose unavailable")
        return Rotation.from_rotvec(r.ravel()), t.ravel()
    base, center = solve(pixels(np.median(corners[times<0],axis=0)))
    out = []
    for q in corners:
        uv = pixels(q)
        absolute, pos = solve(uv)
        initial = np.r_[(base.inv()*absolute).as_rotvec(),base.inv().apply(pos-center)]
        def unpack(v):
            rot = Rotation.from_rotvec(v[:3])
            z = 22.15*(abs(rot.as_matrix()[2]).sum()-1) if on_shelf else v[5]
            return rot,np.array([v[3],v[4],z])
        def residual(v):
            rot, delta = unpack(v)
            cam = base.apply(rot.apply(marker)+delta)+center
            return (cam[:,:2]/cam[:,2,None]*focal+c-uv).ravel()
        fit = least_squares(residual,initial[:5] if on_shelf else initial)
        rot, delta = unpack(fit.x)
        out.append((rot,delta,np.sqrt(np.sum(fit.fun**2)/4)))
    return out


def main():
    solutions = {(lens,support):reconstruct(k,f,support)
                 for lens,k,f in (("See3CAM",-1.499,473.11),("pinhole",0,700))
                 for support in (False,True)}
    with (here/"optical-endpoint.csv").open("w") as f:
        writer = csv.writer(f,lineterminator="\n")
        writer.writerow(["optics","shelf_support","forward_mm","left_mm","rise_mm","frames"])
        for (lens,support), poses in solutions.items():
            settled = [pose[1] for t,pose in zip(rows["sec"],poses) if t >= 9]
            writer.writerow([lens,support,*np.median(settled,axis=0),len(settled)])
    results = []
    anchor = None
    def inspect(model,data,t,lift):
        nonlocal anchor
        if anchor is None:
            q = data.joint("cube_root").qpos.copy()
            anchor = (Rotation.from_quat(q[3:],scalar_first=True),q[:3])
        idx = int(np.argmin(abs(rows["sec"]-t)))
        if min(abs(t-5.5),abs(t-6.0))>.0001:
            return
        r0, p0 = anchor
        for (lens,support),poses in solutions.items():
            rot,delta,rms = poses[idx]
            snapshot = mj.MjData(model)
            snapshot.qpos[:] = data.qpos
            dst = snapshot.joint("cube_root").qpos
            dst[:3] = p0+r0.apply(delta)*.001
            dst[3:] = (r0*rot).as_quat(scalar_first=True)
            mj.mj_forward(model,snapshot)
            near = closest(model,snapshot)
            for side in ("right","left"):
                gap,material = min((near[(side,m)][0]*1000,m) for m in ("rubber","shell"))
                results.append((t,lens,support,side,*delta,rms,gap,material))
    run((1,"stl","overhead","weld",0,0,0),inspect)
    with (here/"optical-clearance.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow(["replay_sec","optics","shelf_support","side","forward_mm","left_mm",
                         "rise_mm","reprojection_rms_px","signed_gap_mm","material"])
        writer.writerows(results)
    fig,axes=plt.subplots(1,2,figsize=(9,4),sharey=True)
    for ax,t in zip(axes,(5.5,6.0)):
        for side,offset in (("right",-.08),("left",.08)):
            q=[r for r in results if r[0]==t and r[3]==side]
            ax.scatter(np.arange(len(q))+offset,[r[-2] for r in q],label=side)
        ax.axhline(0,color="gray",lw=.8)
        ax.set(xticks=range(4),xticklabels=["See3CAM\nfree","See3CAM\nshelf","Pinhole\nfree","Pinhole\nshelf"],
               title=f"{t:.2f} seconds")
    axes[0].set_ylabel("Signed fork clearance (mm)")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(here/"optical-clearance.png",dpi=150)

if __name__=="__main__":
    main()

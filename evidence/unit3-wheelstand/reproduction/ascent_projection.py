"""Project relative W8 cube motion onto independently registered movie patches."""
import csv
from itertools import product
import sys
from unittest.mock import patch

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

from ascent_cube import PATCHES
from dock_outline import glyph_edges
from fork_projection import FORK, dock_camera, intersect, project, rot
from wheel_authority import here, replay, servo


def trajectory(tr, fixture, release):
    v = replay.params.values(replay.params.load(model="vector_2"), {"tip_supports_driven":1})
    m = replay.build(v,fixture)
    m.opt.timestep = .00025
    a = m.actuator("lift")
    a.gainprm[0],a.biasprm[1],a.biasprm[2] = 5,-5,-.03
    sample = replay._Run.sample
    times, positions, rotations = [],[],[]

    def record(r,i,t):
        sample(r,i,t)
        times.append(t-tr.late)
        positions.append(r.d.xpos[r.cube_id].copy()*1000)
        rotations.append(r.d.xmat[r.cube_id].reshape(3,3).copy())

    with patch.object(replay._Run,"sample",record):
        servo(m,v,tr,release,float(m.actuator("left_wheel").gainprm[0]),1.5)
    return np.array(times),np.array(positions),np.array(rotations)


def predicted(tr, cam, traj, t, height):
    a,b = intersect(FORK,93,cam)
    forward = np.array([a[1]-b[1],b[0]-a[0]])
    lens,roof = intersect(np.array([[790.,683.],[732.,651.]]),65.5,cam)
    if forward @ (roof-lens)<0: forward *= -1
    world = np.eye(3)
    world[:2,:2] = rot(np.arctan2(forward[1],forward[0])-tr.obs(3.065,"yaw"))
    ts,xyz,orient = traj
    ix = abs(ts[:,None]-t).argmin(axis=0)
    assert np.max(abs(ts[ix]-t)) < .001
    centre = np.r_[intersect(np.array([[322.,337.]])*2,44,cam)[0],44]
    offset = centre - world @ (xyz[ix[0]]+orient[ix[0]] @ [0,0,22])
    uv = np.array([PATCHES[k] for k in ("left rim","top rim")])
    initial = np.column_stack((intersect(uv*2,height,cam),[height,height]))
    local = ((initial-offset) @ world-xyz[ix[0]]) @ orient[ix[0]]
    output = np.stack([project((local@orient[i].T+xyz[i])@world.T+offset,cam)/2 for i in ix])
    np.testing.assert_allclose(output[0],uv,atol=1e-7)
    return output-output[0]


def main(fixture):
    tr = replay.Trial("W8")
    data = list(csv.DictReader((here/"ascent-cube.csv").open()))
    frames = sorted({int(r["frame"]) for r in data if 91<=int(r["frame"])<=103})
    def observed(key):
        return np.array([np.mean([[float(r[c]) for c in ("dx_px","dy_px")] for r in data
                                  if int(r["frame"])==i and r["patch"]==key],axis=0) for i in frames])
    t = np.array([float(next(r["seconds"] for r in data if int(r["frame"])==i)) for i in frames])
    drift = (observed("wall screw")+observed("dock outline"))/2
    obs = np.stack([observed(k)-drift for k in ("left rim","top rim")],axis=1)
    obs -= obs[:1].copy()
    corners = glyph_edges(cv.imread(str(here.parent/"W8/precontact-crop.jpg"),cv.IMREAD_GRAYSCALE))
    rows = []
    fig,axes = plt.subplots(2,2,figsize=(10,6),sharex=True,layout="constrained")
    for j,key in enumerate(("left rim","top rim")):
        for k,axis in enumerate(("x","y")):
            axes[k,j].plot(t,obs[:,j,k],"ko-",ms=3,label="Physical")
            axes[k,j].set_title(f"{key} · {axis} displacement (px)")
    for release in (3.5,3.8):
        traj = trajectory(tr,fixture,release)
        for focal in (670,675,695):
            cam = dock_camera(focal,corners)
            for height,shift in product((38,44),(-.04,0,.04)):
                pred = predicted(tr,cam,traj,t+shift,height)
                if (focal,height,shift)==(675,44,0):
                    for j,k in product(range(2),repeat=2):
                        axes[k,j].plot(t,pred[:,j,k],".--",label=f"Release {release} s")
                for i0,i1 in ((91,103),(95,98)):
                    sel=(np.array(frames)>=i0)&(np.array(frames)<=i1)
                    residual=(pred-obs)[sel]
                    residual -= residual[:1].copy()
                    rms=np.sqrt(np.mean(np.sum(residual[1:]**2,axis=2)))
                    rows.append([release,focal,height,shift,i0,i1,rms])
    for ax in axes[1]: ax.set_xlabel("Seconds from last raised-lift sample")
    axes[0,0].legend(fontsize=8)
    fig.savefig(here/"ascent-projection.png",dpi=160)
    with (here/"ascent-projection.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("release focal_px landmark_height_mm time_shift_s first_frame last_frame radial_rms_px".split())
        w.writerows(rows)


if __name__=="__main__": main(sys.argv[1])

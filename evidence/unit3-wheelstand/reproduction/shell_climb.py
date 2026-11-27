"""Replay from before cube contact with the candidate rear shell."""
import csv
import sys
from pathlib import Path
from unittest.mock import patch

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull

from corner_grip import dock_camera, glyph_edges, predicted, servo
from encoder_follow import evaluate
from shell_settle import here, rear_model, replay, shell_triangles


def camera_reference():
    with (here/"ascent-cube.csv").open() as file:
        rows = list(csv.DictReader(file))
    times, observed = [], []
    for frame in range(91,104):
        group = [r for r in rows if int(r["frame"]) == frame]
        times.append(float(group[0]["seconds"]))
        def point(name):
            return np.mean([[float(r["dx_px"]),float(r["dy_px"])] for r in group if r["patch"] == name],axis=0)
        drift = (point("wall screw")+point("dock outline"))/2
        observed.append([point(key)-drift for key in ("left rim","top rim")])
    cam = dock_camera(675,glyph_edges(cv.imread(str(here.parent/"W8/precontact-crop.jpg"),0)))
    return cam,np.array(times),np.array(observed)


def simulate(v, fixture, rear, geometry, tr, drive, start):
    model = replay.build(v,fixture) if geometry == "Box" else rear_model(v,fixture,rear,geometry == "Shell 10.75")
    model.opt.timestep = .00025
    lift = model.actuator("lift")
    lift.gainprm[0],lift.biasprm[1],lift.biasprm[2] = 5,-5,-.03
    sample = replay._Run.sample
    times,positions,rotations,loaded = [],[],[],[]
    shell = -1 if geometry == "Box" else model.geom("rear_surface").id
    table = model.geom("table").id
    wrench = np.zeros(6)
    def capture(run,i,t):
        sample(run,i,t)
        times.append(t-tr.late)
        positions.append(run.d.xpos[run.cube_id].copy()*1000)
        rotations.append(run.d.xmat[run.cube_id].reshape(3,3).copy())
        for j,c in enumerate(run.d.contact):
            if set(c.geom) == {shell,table}:
                replay.mj.mj_contactForce(model,run.d,j,wrench)
                if wrench[0] > .001:
                    loaded.append(t-tr.late)
                    break
    with patch.object(replay._Run,"sample",capture):
        if drive == "Measured wheels":
            trace,_,score = servo(model,v,tr,start,float(model.actuator("left_wheel").gainprm[0]),1.5)
        else:
            trace,score = evaluate(model,v,tr,start,tr.late if drive.endswith("late") else tr.early,True)
    return trace,score,(np.array(times),np.array(positions),np.array(rotations)),loaded


def main(fixture,rig):
    v = replay.params.values(replay.params.load(model="vector_2"),{"tip_supports_driven":1})
    vertices = np.unique(shell_triangles(Path(rig),v).reshape(-1,3),axis=0)
    rear = vertices[vertices[:,0]<-v["wheel_base_mm"]]
    rear = rear[ConvexHull(rear).vertices]
    cam,ts,obs = camera_reference()
    fig,axes = plt.subplots(3,3,figsize=(13,9),layout="constrained")
    rows = []
    for k,drive in enumerate(("Measured wheels","Pitch cutoff early","Pitch cutoff late")):
        for col,name in enumerate(("W4","W8")):
            tr = replay.Trial(name)
            t = tr.state["sec"]-tr.zero-tr.late
            view = (t>=-.16)&(t<=.82)
            ax = axes[k,col]
            ax.plot(t[view],np.degrees(tr.state["pitch"][view]),"ko-",ms=3,label="Recorded",zorder=4)
            for idx,geometry in enumerate(("Box","Shell 10.45","Shell 10.75")):
                for start in (3.5,3.8):
                    trace,score,traj,loaded = simulate(v,fixture,rear,geometry,tr,drive,start)
                    label = geometry if start==3.5 else None
                    style = dict(color=f"C{idx}",ls="-" if start==3.5 else ":",lw=1.3,label=label)
                    ax.plot(trace[:,0]-tr.late,trace[:,1],**style)
                    err = ["",""]
                    if name == "W8":
                        est = predicted(tr,cam,traj,ts,44)
                        axes[k,2].plot(ts,est[:,0,0]-est[0,0,0],**style)
                        for n,sl in enumerate((slice(None),slice(4,8))):
                            residual = (est-obs)[sl]
                            err[n] = np.mean(np.sum((residual[1:]-residual[0])**2,axis=2))**.5
                    rows.append([name,drive,geometry,start,*score[:3],min(loaded) if loaded else "",*err])
            ax.set(xlim=(-.16,.8),title=f"{name} · {drive}",ylabel="Pitch (degrees)")
        axes[k,2].plot(ts,obs[:,0,0]-obs[0,0,0],"ko-",ms=3)
        axes[k,2].set(title="W8 cube left rim",ylabel="Horizontal movement (px)")
    for ax in axes[-1]: ax.set_xlabel("Seconds from raised-lift sample")
    axes[0,0].legend(fontsize=7)
    fig.savefig(here/"shell-climb.png",dpi=150)
    with (here/"shell-climb.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow("run drive geometry release lift_rmse_mm peak_deg held_deg first_shell_load_s w8_whole_px w8_ascent_px".split())
        writer.writerows(rows)


if __name__ == "__main__":
    main(*sys.argv[1:])

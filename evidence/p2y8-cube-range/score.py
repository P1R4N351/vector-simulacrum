"""Replay archived shelf pushes with the production ToF fan and reporting logic.

Run with repository root and registered fork XML as positional arguments.
"""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(HERE.parent / "p2y8-printed-fork-contact"), str(ROOT)]
import evaluate
from simulacrum import params, prox


def replay_trial(trial):
    states = evaluate.states[evaluate.states["trial"] == trial]
    values = params.values(params.load(model="vector_2"))
    sensor = prox.ProxSensor(values, noise_seed=trial)
    period = values["state_period_ms"] / 1000
    tilt = np.deg2rad(values["tof_tilt_up_deg"])
    c, s = np.cos(tilt), np.sin(tilt)
    rays = np.asarray(sensor.rays) @ np.array([[s, 0, -c], [0, 1, 0], [c, 0, s]])
    samples = []
    next_sample = 0

    def observe(model, data, relative, lift):
        nonlocal next_sample
        t = relative + 1.5
        if t + 1e-8 < next_sample:
            return
        next_sample += period
        x, y, yaw = [np.interp(t, states["sec"], states[k]) for k in ("x_mm","y_mm","yaw_deg")]
        h = np.deg2rad(yaw)
        rot = np.array([[np.cos(h),-np.sin(h),0],[np.sin(h),np.cos(h),0],[0,0,1]])
        origin = (rot @ values["tof_position_mm"] + [x,y,0]) / 1000
        # Group the entire fork assembly separately even if the source XML mixes render groups.
        cube = model.body("cube").id
        model.geom_group[:] = (model.geom_bodyid != cube) & (model.geom_bodyid != 0)
        scene = np.array([1,0,0,0,0,0], dtype=np.uint8)
        gid = np.zeros(1,dtype=np.int32)
        hits = [1000*mujoco.mj_ray(model,data,origin,rot @ ray,scene,1,-1,gid) for ray in rays]
        reading = sensor.read(hits,lift,0.0)
        samples.append((t,reading.distance_mm,reading.found_object))

    evaluate.run((trial,"stl","overhead","weld",0,0,0),observe)
    samples = np.asarray(samples)
    rows = []
    for r in states[states["sec"] >= 0]:
        j = max(0,np.searchsorted(samples[:,0],r["sec"],side="right")-1)
        rows.append((trial,r["sec"],r["lift_mm"],r["prox_mm"],int(r["found"]),samples[j,1],int(samples[j,2])))
    return np.asarray(rows)


if __name__ == "__main__":
    fig, axes = plt.subplots(5,1,sharex=True,figsize=(9,8))
    with (HERE/"samples.csv").open("w") as out:
        writer = csv.writer(out,lineterminator="\n")
        writer.writerow(["trial","seconds","lift_mm","real_mm","real_found","sim_mm","sim_found"])
        for trial, ax in enumerate(axes,1):
            rows = replay_trial(trial)
            writer.writerows(rows)
            for idx,label,color in [(3,"P2Y8","#116d84"),(5,"Replay","#b95a2c")]:
                valid = rows[:,idx+1].astype(bool)
                ax.plot(rows[:,1],np.where(valid,rows[:,idx],np.nan),label=label,color=color)
                ax.scatter(rows[~valid,1],rows[~valid,idx],marker="x",s=8,color=color)
            ax.set_ylabel(f"Trial {trial}\nmm")
            both = (rows[:,4]*rows[:,6]).astype(bool)
            err = rows[both,5]-rows[both,3]
            print(trial,len(rows),both.sum(),np.mean(np.abs(err)),np.mean(err),np.mean(rows[:,4]==rows[:,6]))
    axes[0].legend(ncol=2)
    axes[-1].set_xlabel("Seconds from replay start")
    fig.suptitle("Cube range during five shelf pushes · crosses mark found_object false")
    fig.tight_layout()
    fig.savefig(HERE/"range.png",dpi=150)

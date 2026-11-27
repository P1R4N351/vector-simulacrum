"""Check whether both measured wheel trajectories recover the wheelstand with recorded lift travel."""
import csv
import itertools
import sys
from unittest.mock import patch

import matplotlib.pyplot as plt
import numpy as np

from encoder_follow import evaluate, here, replay


def trial(model, v, tr, release, onset, follow):
    original = replay._Run.control
    samples = []

    def control(run, t):
        # Override the inputs to the existing tread loops; update each loop exactly once.
        with patch.object(run.controllers[0], "update", side_effect=target(run.controllers[0].update, t, "left")), \
             patch.object(run.controllers[1], "update", side_effect=target(run.controllers[1].update, t, "right")):
            original(run, t)
        samples.append([t, *[run.d.joint("wheel_f"+s+"_joint").qvel[0]*v["wheel_front_radius_mm"] for s in "lr"]])

    def target(update, t, side):
        def apply(requested, measured):
            return update(tr.obs(t, side) if follow and t >= onset else requested, measured)
        return apply

    with patch.object(replay._Run, "control", control):
        trace, scores = evaluate(model, v, tr, release, onset, True)
    return trace, np.asarray(samples), scores


def main(fixture):
    v = replay.params.values(replay.params.load(model="vector_2"), {"tip_supports_driven":1})
    model = replay.build(v, fixture)
    model.opt.timestep = .00025
    lift = model.actuator("lift")
    lift.gainprm[0] = 5
    lift.biasprm[1:3] = [-5,-.03]
    fig, axes = plt.subplots(3,2,figsize=(10,7),sharex=True,sharey="row",layout="constrained")
    rows = []
    for col, name in enumerate(("W4","W8")):
        tr = replay.Trial(name)
        t = tr.state["sec"] - tr.zero
        valid = (t >= tr.late) & (t <= tr.late + .8)
        obs = np.stack([tr.state[s][valid] for s in ("left","right")])
        a = axes[:,col]
        for ax, y in zip(a, (*obs, np.degrees(tr.state["pitch"][valid]))):
            ax.plot(t[valid]-tr.late,y,"ko-",ms=3,label="Physical",zorder=3)
        for follow in (False,True):
            for case,(release,onset) in enumerate(itertools.product((3.5,3.8),(tr.early,tr.late)),1):
                trace,w,scores = trial(model,v,tr,release,onset,follow)
                est = [np.interp(t[valid],w[:,0],w[:,i]) for i in (1,2)]
                rms = np.sqrt(np.mean((est-obs)**2,axis=1))
                stopped = np.all(obs == 0,axis=0)
                moving = np.mean(obs,axis=0) >= 100
                residual = np.asarray(est) - obs
                rows.append([name,case,follow,*rms,*scores,np.mean(residual[:,moving]),
                             np.mean(np.abs(residual[:,stopped]))])
                for ax, times, y in zip(a,(w[:,0],w[:,0],trace[:,0]),(w[:,1],w[:,2],trace[:,1])):
                    ax.plot(times-tr.late,y,color="C1" if follow else "C0",alpha=.5,
                            label=("SDK wheel targets" if follow else "Pitch cutoff") if case==1 else None)
        a[0].set_title(name)
        a[2].set(xlim=(-.04,.8),xlabel="Seconds from last raised-lift sample")
    for ax,label in zip(axes[:,0],("Left wheel (mm/s)","Right wheel (mm/s)","Chassis pitch (°)")):
        ax.set_ylabel(label)
    axes[0,0].legend(fontsize=7)
    fig.savefig(here/"wheel-follow.png",dpi=160)
    with (here/"wheel-follow.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow("run case follow left_rmse_mmps right_rmse_mmps lift_rmse_mm peak_pitch_deg held_pitch_deg max_torque_mNm moving_bias_mmps stopped_mae_mmps".split())
        writer.writerows(rows)


if __name__=="__main__": main(sys.argv[1])

"""Drive only the lift toward its recorded trajectory and observe the free chassis."""
import csv
import itertools
import sys
from unittest.mock import patch

import matplotlib.pyplot as plt
import numpy as np

from acceleration_receipts import here, replay


def evaluate(m, v, tr, release, onset, tracking):
    original = replay._Run.control
    torque = []
    act = m.actuator("lift")
    start = release + tr.shift
    t_sdk = tr.state["sec"] - tr.zero

    def control(run, t):
        original(run, t)
        if tracking and t >= tr.early:
            goal = np.interp(t, t_sdk, tr.state["lift"])
            run.d.actuator("lift").ctrl[0] = replay.mjcf.lift_angle_of_height(v, goal)

    def record(run, t):
        if tr.late <= t <= tr.late + .2:
            torque.append(abs(run.d.actuator("lift").force[0]) * 1000)

    with patch.object(replay._Run, "control", control):
        trace, *_ = replay.run(m, v, tr, start, onset, "servo:300" if tracking else "servo:30", 17, on_step=record)
    sample = (t_sdk >= tr.late) & (t_sdk <= tr.late + .2)
    estimates = np.interp(t_sdk[sample], trace[:,0], trace[:,4])
    rms = np.mean((estimates - tr.state["lift"][sample])**2)**.5
    window = (trace[:,0] >= tr.late) & (trace[:,0] <= tr.late + .8)
    hold = window & (trace[:,0] >= tr.late + .4)
    return trace, [rms, trace[window,1].max(), np.median(trace[hold,1]), max(torque)]


def main(fixture):
    v = replay.params.values(replay.params.load(model="vector_2"), {"tip_supports_driven":0})
    rows = []
    fig, axes = plt.subplots(2,2,figsize=(10,6),layout="constrained",sharex=True,sharey="row")
    for col, name in enumerate(("W4","W8")):
        tr = replay.Trial(name)
        sec = tr.state["sec"] - tr.zero - tr.late
        view = (sec >= -.20) & (sec <= .85)
        for panel, data in zip(axes[:,col], (tr.state["lift"], np.degrees(tr.state["pitch"]))):
            panel.plot(sec[view], data[view], "ko-", label="Physical SDK", ms=3,zorder=4)
        for driven, tracking in itertools.product((False, True), repeat=2):
            v["tip_supports_driven"] = int(driven)
            label = ("Driven" if driven else "Fixed") + (" · encoder" if tracking else " · baseline")
            m = replay.build(v,fixture)
            m.opt.timestep = .00025
            if tracking:
                act = m.actuator("lift")
                # A deliberately strong virtual position servo. Applied torque is reported.
                act.gainprm[0] = 5
                act.biasprm[1:3] = [-5,-.03]
            for case,(release,onset) in enumerate(itertools.product((3.5,3.8),(tr.early,tr.late)),1):
                trace,score = evaluate(m,v,tr,release,onset,tracking)
                rows.append([name,case,driven,tracking,*score])
                for panel,idx in zip(axes[:,col],(4,1)):
                    panel.plot(trace[:,0]-tr.late,trace[:,idx],color=f"C{2*driven+tracking}",alpha=.6,
                               label=label if case==1 else None)
        axes[0,col].set_title(name)
        axes[1,col].set_xlabel("Seconds from last raised-lift sample")
        axes[1,col].set_xlim(-.18,.80)
    axes[0,0].set_ylabel("Lift height (mm)")
    axes[1,0].set_ylabel("Chassis pitch (°)")
    axes[0,0].legend(fontsize=8)
    fig.savefig(here/"encoder-follow.png",dpi=160)
    with (here/"encoder-follow.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow("run case driven tracking early_lift_rmse_mm peak_pitch_deg held_pitch_deg max_early_torque_mNm".split())
        writer.writerows(rows)


if __name__=="__main__": main(sys.argv[1])

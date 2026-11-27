"""Resolve the cube's vertical impulse on the fork and chassis during ascent."""
import csv
import itertools
import sys

import matplotlib.pyplot as plt
import numpy as np

from acceleration_receipts import here, replay


def measure(m, v, tr, start, onset):
    ids = m.geom_bodyid
    cube = m.body("cube").id
    fork = m.body("lift_fork").id
    wrench = np.empty(6)
    impulse = np.zeros(2)

    def contacts(r, t):
        if not tr.late < t <= tr.late + .20:
            return
        for i, c in enumerate(r.d.contact):
            a, b = ids[c.geom]
            if cube not in (a, b) or 0 in (a, b):
                continue
            # mj_contactForce acts on geom 2, in contact coordinates.
            replay.mj.mj_contactForce(m, r.d, i, wrench)
            vertical = c.frame.reshape(3, 3)[:, 2] @ wrench[:3]
            robot = b if a == cube else a
            impulse[int(robot != fork)] += (vertical if a == cube else -vertical) * r.dt

    trace, *_ = replay.run(m, v, tr, start, onset, "servo:30", 17, on_step=contacts)
    t = tr.state["sec"] - tr.zero
    early = (t >= tr.late) & (t <= tr.late + .20)
    lift = np.interp(t[early], trace[:, 0], trace[:, 4])
    lift_error = np.sqrt(np.mean((lift - tr.state["lift"][early])**2))
    peak = np.max(trace[(trace[:, 0] >= tr.late) & (trace[:, 0] <= tr.late + .8), 1])
    return [*impulse*1000, lift_error, peak, int(early.sum())]


def main(fixture):
    rows = []
    v = replay.params.values(replay.params.load(model="vector_2"), {"tip_supports_driven": 0})
    m = replay.build(v, fixture)
    fig, axes = plt.subplots(2, 3, figsize=(11, 6), layout="constrained", sharex=True)
    steps = [.25, .125, .0625]
    for ax, name in zip(axes, ("W4", "W8")):
        tr = replay.Trial(name)
        for case, (release, onset) in enumerate(itertools.product((3.5, 3.8), (tr.early, tr.late)), 1):
            results = []
            for step in steps:
                m.opt.timestep = step * .001
                result = measure(m, v, tr, release + tr.shift, onset)
                rows.append([name, case, step, *result])
                results.append(result)
            for panel, idx in zip(ax, (0, 2, 3)):
                panel.plot(steps, np.array(results)[:, idx], ".-", label=f"Timing {case}")
                panel.set_xticks(steps, labels=["¼", "⅛", "¹⁄₁₆"])
        ax[-1].axhline(tr.physical()["peak_pitch_deg"], color="k", ls="--", label="Physical")
        for panel, quantity in zip(ax, ("Cube → fork vertical impulse (mN s)", "Lift travel RMS error (mm)", "Peak chassis pitch (°)")):
            panel.set_title(name + " · " + quantity, fontsize=10)
            if not panel.xaxis_inverted(): panel.invert_xaxis()
    for panel in axes[-1]:
        panel.set_xlabel("Physics step (ms)")
    axes[0, -1].legend(fontsize=7)
    fig.savefig(here / "load-transfer.png", dpi=160)
    with (here / "load-transfer.csv").open("w") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow("run case dt_ms cube_fork_up_mNs cube_other_robot_up_mNs lift_rmse_mm peak_pitch_deg physical_lift_samples".split())
        writer.writerows(rows)


if __name__ == "__main__":
    main(sys.argv[1])

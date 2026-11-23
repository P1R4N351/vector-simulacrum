"""Impose the recorded lift stroke as a diagnostic input to the free wheelstand."""

import argparse
import csv
import hashlib
from pathlib import Path
from unittest.mock import patch

import matplotlib.pyplot as plt
import numpy as np

from support_wheelstand import HERE, mjcf, params, replay


class InstrumentedStroke(replay._Run):
    def sample(self, i, t):
        super().sample(i, t)
        self.trace[-1].append(float(self.d.actuator_force[self.m.actuator("lift").id]))


class RecordedStroke(InstrumentedStroke):
    strong = False

    def control(self, t):
        super().control(t)
        if t < self.onset:
            return
        act = self.m.actuator("lift")
        if self.strong:
            # Diagnostic high-gain drive; this torque envelope is not a motor estimate.
            act.gainprm[0], act.biasprm[1], act.biasprm[2] = 12, -12, -0.035
            act.forcerange[:] = [-0.5, 0.5]
        height = self.tr.obs(t, "lift")
        self.d.ctrl[act.id] = mjcf.lift_angle_of_height(self.v, height)


class AssistedStroke(RecordedStroke):
    strong = True


# First descending 40 mm crossing; compare pitch at matched lift displacement.
def crossing_at_40_mm(time, h, p):
    after = np.flatnonzero((time[:-1] >= 0) & (h[:-1] > 40) & (h[1:] <= 40))
    if not len(after):
        return [np.nan, np.nan]
    j = after[0]
    f = (h[j] - 40) / (h[j] - h[j + 1])
    return [time[j] + f * (time[j + 1] - time[j]), p[j] + f * (p[j + 1] - p[j])]


def main(fixture):
    assert (
        hashlib.sha256(fixture.read_bytes()).hexdigest()
        == "4867e81c8d8d92ad3abc621d920af9b550c001e5d8f81d8443d1e84a069b8230"
    )
    v = params.values(params.load(model="vector_2"))
    fig, axes = plt.subplots(
        3, 2, figsize=(11, 8), layout="constrained", sharex="col", sharey="row"
    )
    rows = []
    for axs, name in zip(axes.T, ["W4", "W8"]):
        tr = replay.Trial(name)
        t = tr.state["sec"] - tr.zero - tr.late
        pitch = np.degrees(tr.state["pitch"])
        height = tr.state["lift"]
        use = (t >= -0.1) & (t <= 0.8)
        axs[0].plot(t[use], height[use], "ko-", ms=3, label="Recorded", zorder=4)
        axs[1].plot(t[use], pitch[use], "ko-", ms=3, zorder=4)
        ct = tr.cube["sec"] - tr.zero - tr.late
        use_cube = (ct >= -0.1) & (ct <= 0.8)
        acc = np.linalg.norm(np.stack([tr.cube[k] for k in "xyz"]), axis=0) / 8192
        axs[2].plot(ct[use_cube], acc[use_cube], "k.-", lw=1, zorder=4)
        for label, driver, color in [
            ("Profile", InstrumentedStroke, "#986225"),
            ("Recorded target", RecordedStroke, "#267b7a"),
            ("Assisted tracking", AssistedStroke, "#7048a5"),
            ("Assisted + four driven", AssistedStroke, "#266591"),
        ]:
            for index, start in enumerate([3.5 + tr.shift, 3.8 + tr.shift]):
                # New model per case because the diagnostic changes gains after onset.
                model_values = (
                    dict(v, tip_supports_driven=1)
                    if label == "Assisted + four driven"
                    else v
                )
                m = replay.build(model_values, fixture)
                m.opt.timestep = v["sim_timestep_ms"] * 0.001 / 4
                with patch.object(replay, "_Run", driver):
                    trace, pk, taps, _ = replay.run(
                        m, model_values, tr, start, tr.late, "servo:60", 17
                    )
                rel = trace[:, 0] - tr.late
                early = (t >= 0) & (t <= 0.25)
                lift_rmse = np.sqrt(
                    np.mean(
                        (np.interp(t[early], rel, trace[:, 4]) - height[early]) ** 2
                    )
                )
                sc = replay.score(tr, trace, pk, taps, tr.late)
                tc, pc = crossing_at_40_mm(rel, trace[:, 4], trace[:, 1])
                rt, rp = crossing_at_40_mm(t, height, pitch)
                torque = np.max(abs(trace[(rel >= 0) & (rel <= 0.25), -1]))
                rows.append(
                    [
                        name,
                        label,
                        start,
                        lift_rmse,
                        rt,
                        rp,
                        tc,
                        pc,
                        sc["peak_pitch_deg"],
                        sc["hold_pitch_deg"],
                        sc["pitch_rmse_deg"],
                        sc["peak_g"],
                        torque,
                    ]
                )
                for ax, time, y in zip(
                    axs,
                    [rel, rel, pk[:, 0] - tr.late],
                    [trace[:, 4], trace[:, 1], pk[:, 1]],
                ):
                    ax.plot(
                        time,
                        y,
                        color=color,
                        lw=1.1,
                        alpha=0.8,
                        label=label if index == 0 else None,
                    )
        axs[0].set_title(name)
        axs[-1].set(xlim=(-0.1, 0.8), xlabel="Seconds from last raised-lift sample")
    for ax, label in zip(
        axes[:, 0], ["Lift height (mm)", "Chassis pitch (°)", "Cube specific force (g)"]
    ):
        ax.set_ylabel(label)
    axes[0, -1].legend(fontsize=8)
    fig.savefig(HERE / "loaded-stroke.png", dpi=160)
    with (HERE / "loaded-stroke.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            [
                "trial",
                "driver",
                "start_s",
                "lift_rmse_mm",
                "real_40_s",
                "real_40_pitch_deg",
                "sim_40_s",
                "sim_40_pitch_deg",
                "peak_deg",
                "hold_deg",
                "pitch_rmse_deg",
                "cube_peak_g",
                "peak_lift_torque_Nm",
            ]
        )
        w.writerows(rows)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("fixture", type=Path)
    main(parser.parse_args().fixture)

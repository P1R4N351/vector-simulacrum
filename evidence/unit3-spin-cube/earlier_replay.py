"""Test the frozen wheelstand replay on seven earlier mounts with visible cube poses."""

import csv
import json
import subprocess

import matplotlib.pyplot as plt
import numpy as np

from earlier_inputs import TRIALS
import mount_replay as m
from tap_timing import events


def main():
    v, derived, source = m._load_model_config()
    poses, path, packets = [
        m._load_structured(f"earlier-{name}.csv.gz")
        for name in ("pose", "path", "packets")
    ]
    fig, axes = plt.subplots(
        2, 4, figsize=(13, 5), layout="constrained", sharex=True, sharey=True
    )
    axes.flat[-1].set_visible(False)
    with (m.HERE / "earlier-timing.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            "trial contact phase_ms physical_events_s simulated_events_s physical_count simulated_count".split()
        )
        for trial, ax in zip(TRIALS, axes.flat):
            pkt = packets[packets["trial"] == trial]
            assert (
                pkt["t_s"][0] < -0.8
                and pkt["t_s"][-1] > 0.1
                and np.all(np.diff(pkt["t_s"]) > 0)
            )
            real = events(pkt["t_s"], pkt["tap"])
            ax.vlines(real, -0.3, 1.3, color="black", lw=1, linestyle="--")
            for row, contact in enumerate(("fork", "full")):
                model = m.build_model(
                    source, v, derived, drive="freejoint", contact=contact
                )
                traj = m.compute_trajectory(
                    path[path["trial"] == trial], v, model.opt.timestep
                )
                sim = m.simulate_trial(
                    model, poses[poses["trial"] == trial][0], traj, drive="freejoint"
                )
                for phase in range(0, 10, 2):
                    i = round(phase / (1000 * sim["dt"]))
                    arg = dict(
                        dt=sim["dt"], start=sim["times"][i], forces=sim["forces"][i:]
                    )
                    cmd = ["node", str(m.HERE / "mount_sensor.mjs")]
                    arr = np.array(
                        json.loads(
                            subprocess.check_output(
                                cmd, input=json.dumps(arg), text=True, cwd=m.HERE
                            )
                        )
                    )
                    est = events(arr[:, 0], arr[:, 4])
                    w.writerow(
                        [
                            trial,
                            contact,
                            phase,
                            json.dumps(real.tolist()),
                            json.dumps(est.tolist()),
                            len(real),
                            len(est),
                        ]
                    )
                    ax.scatter(
                        est,
                        np.full(len(est), row + (phase - 4) * 0.045),
                        marker="|",
                        s=95,
                        color=["teal", "darkorange"][row],
                    )
            ax.set(
                title=f"{trial} · {len(real)} physical",
                yticks=[0, 1],
                yticklabels=["Fork", "Full"],
                xlim=(-0.8, 0.1),
                ylim=(-0.35, 1.35),
            )
    fig.supxlabel("Seconds from first SDK pitch report between 45° and 85°")
    fig.suptitle(
        "Physical tap packets (dashed) and simulated increments at five sampling phases"
    )
    fig.savefig(m.HERE / "earlier-timing.png", dpi=170)


if __name__ == "__main__":
    main()

"""Compare early physical cube acceleration to the frozen fork trajectory and sensor."""

import csv
import json
import subprocess
import matplotlib.pyplot as plt
import numpy as np
from earlier_inputs import TRIALS
import mount_replay as m


def wave(t, xyz, base):
    return np.linalg.norm(xyz - np.median(xyz[base], axis=0), axis=1)


def onset(t, dev, level):
    hits = np.flatnonzero((t >= -2) & (t <= 0.1) & (dev >= level))
    return float(t[hits[0]]) if len(hits) else np.nan


def main():
    poses, path, slots = [
        m._load_structured(file)
        for file in (
            "impact-initial-poses.csv",
            "earlier-path.csv.gz",
            "impact-samples.csv.gz",
        )
    ]
    v, derived, xml = m._load_model_config()
    model = m.build_model(xml, v, derived, drive="freejoint", contact="fork")
    fig, axes = plt.subplots(2, 4, figsize=(14, 6), layout="constrained", sharex=True)
    axes.flat[-1].set_visible(False)
    with (m.HERE / "impact-onset.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            "trial level_g phase_ms physical_s geometric_contact_s simulated_s".split()
        )
        for name, ax in zip(TRIALS, axes.flat):
            pkt = slots[slots["trial"] == name]
            t = pkt["packet_s"]
            xyz = np.stack([pkt[k] for k in "xyz"], axis=1) / 8192
            dev = wave(t, xyz, t < -4)
            traj = m.compute_trajectory(
                path[path["trial"] == name], v, model.opt.timestep
            )
            sim = m.simulate_trial(
                model, poses[poses["trial"] == name][0], traj, drive="freejoint"
            )
            contact = sim["first_fork_contact_s"]
            keep = (t >= -2) & (t <= 0.1)
            ax.scatter(
                t[keep], dev[keep], color="black", s=5, label="Physical slots", zorder=3
            )
            ax.axvline(contact, color="tab:red", ls="--", label="First fork contact")
            for phase in range(0, 10, 2):
                i = round(phase / (1000 * sim["dt"]))
                args = dict(
                    dt=sim["dt"], start=sim["times"][i], forces=sim["forces"][i:]
                )
                arr = np.array(
                    json.loads(
                        subprocess.check_output(
                            ["node", str(m.HERE / "mount_sensor.mjs")],
                            input=json.dumps(args),
                            text=True,
                        )
                    )
                )
                st = arr[:, 0]
                sd = wave(st, arr[:, 1:4], (st >= -2.9) & (st < -2.1))
                if phase == 0:
                    mask = (st >= -2) & (st <= 0.1)
                    ax.scatter(
                        st[mask],
                        sd[mask],
                        s=4,
                        color="teal",
                        alpha=0.6,
                        label="Simulated slots (0 ms)",
                    )
                for level in (0.05, 0.1, 0.2):
                    w.writerow(
                        [
                            name,
                            level,
                            phase,
                            onset(t, dev, level),
                            contact,
                            onset(st, sd, level),
                        ]
                    )
            ax.set(title=name, yscale="symlog", ylim=(-0.003, 12))
            ax.set_yscale("symlog", linthresh=0.04)
    axes.flat[0].legend(fontsize=7)
    fig.supylabel("Acceleration change from baseline (g)")
    fig.supxlabel("Seconds from first SDK pitch report between 45° and 85°")
    fig.savefig(m.HERE / "impact-onset.png", dpi=160)


if __name__ == "__main__":
    main()

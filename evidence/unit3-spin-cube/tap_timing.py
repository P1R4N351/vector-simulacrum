"""Compare first tap timing and excess taps on the existing physical mount replay."""

import csv
import json
import subprocess

import matplotlib.pyplot as plt
import numpy as np

import mount_replay as replay

START, END = -0.8, 0.1


def events(time, counter):
    # Keep only the first slot of each packet, whose three slots share a counter.
    first = np.r_[True, np.diff(time) != 0]
    time, counter = time[first], counter[first].astype(int)
    delta = np.diff(counter) % 256
    keep = (time[1:] >= START) & (time[1:] <= END)
    return np.repeat(time[1:][keep], delta[keep])


def main():
    p = replay.HERE
    config, derived, xml = replay._load_model_config()
    path, placements, packets = (
        replay._load_structured(f)
        for f in ("path.csv.gz", "observations.csv", "cube.csv.gz")
    )
    models = {
        name: replay.build_model(xml, config, derived, drive="freejoint", contact=name)
        for name in ("fork", "full")
    }
    fig, axes = plt.subplots(1, 3, figsize=(11, 4.5), sharey=True, layout="constrained")
    with (p / "tap-timing.csv").open("w") as out:
        writer = csv.writer(out, lineterminator="\n")
        writer.writerow(
            [
                "trial",
                "contact",
                "pose",
                "phase_ms",
                "physical_events_s",
                "sim_events_s",
                "first_error_ms",
                "sim_taps",
            ]
        )
        for ax, trial in zip(axes, replay.TRIALS):
            phys = packets[packets["trial"] == trial]
            real = events(phys["packet_s"], phys["tap"])
            assert len(real) == 1
            ax.axvline(real[0], color="black", label="Physical counter increment")
            labels = []
            for name, model in models.items():
                traj = replay.compute_trajectory(
                    path[path["trial"] == trial], config, model.opt.timestep
                )
                for obs in placements[placements["trial"] == trial]:
                    sim = replay.simulate_trial(model, obs, traj, drive="freejoint")
                    labels.append(f"{name}, {obs['observation']}")
                    for phase_ms in range(0, 10, 2):
                        offset = round(phase_ms / (1000 * sim["dt"]))
                        arg = dict(
                            dt=sim["dt"],
                            start=sim["times"][offset],
                            forces=sim["forces"][offset:],
                        )
                        result = subprocess.check_output(
                            ["node", str(p / "mount_sensor.mjs")],
                            input=json.dumps(arg),
                            text=True,
                            cwd=p,
                        )
                        arr = np.array(json.loads(result))
                        est = events(arr[:, 0], arr[:, 4])
                        writer.writerow(
                            [
                                trial,
                                name,
                                obs["observation"],
                                phase_ms,
                                json.dumps(real.tolist()),
                                json.dumps(est.tolist()),
                                round(1000 * (est[0] - real[0]), 1) if len(est) else "",
                                len(est),
                            ]
                        )
                        ax.scatter(
                            est,
                            np.full(
                                len(est), len(labels) - 1 + 0.08 * (phase_ms / 2 - 2)
                            ),
                            marker="|",
                            s=70,
                            color=("teal" if name == "fork" else "darkorange"),
                        )
            ax.set(
                title=trial,
                xlim=(START, END),
                xlabel="Seconds from detected stand",
                yticks=range(4),
                yticklabels=labels,
            )
    axes[0].legend(loc="lower left", fontsize=8)
    fig.savefig(p / "tap-timing.png", dpi=180)


if __name__ == "__main__":
    main()

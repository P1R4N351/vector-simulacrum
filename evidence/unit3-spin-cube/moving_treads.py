"""Paired zero-speed and measured-speed trials with identical front rotor assemblies."""

from pathlib import Path
from tempfile import TemporaryDirectory
import csv
import numpy as np
import matplotlib.pyplot as plt
from actuated_mount import replay
from earlier_inputs import HERE, TRIALS
import mount_replay as m
from tap_timing import events


def main():
    wheels = m._load_structured("approach-wheels.csv.gz")
    physical = m._load_structured("impact-samples.csv.gz")
    fig, axes = plt.subplots(
        2, 4, figsize=(13, 6), sharex=True, sharey=True, layout="constrained"
    )
    axes.flat[-1].set_visible(False)
    with TemporaryDirectory() as temp, (HERE / "moving-treads.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            "trial phase_ms mode physical_count sim_count first_error_ms physical_peak_g sim_peak_g".split()
        )
        for mode in ("Welded", "Stopped", "Measured"):
            inp = wheels.copy()
            if mode == "Stopped":
                inp["left_mmps"] = 0
                inp["right_mmps"] = 0
            metrics = {}

            def observe(name, phase, arr):
                pk = physical[physical["trial"] == name]
                pt = pk["packet_s"]
                p = np.column_stack([pk[k] for k in "xyz"]) / 8192
                pd = np.linalg.norm(p - np.median(p[pt < -4], axis=0), axis=1)
                st = arr[:, 0]
                sd = np.linalg.norm(
                    arr[:, 1:4]
                    - np.median(arr[(st >= -2.9) & (st < -2.1), 1:4], axis=0),
                    axis=1,
                )
                e = events(st, arr[:, 4])
                metrics[name, phase] = (
                    [
                        max(pd[abs(pt - e[0]) <= 0.05], default=""),
                        max(sd[abs(st - e[0]) <= 0.05]),
                    ]
                    if len(e)
                    else ["", ""]
                )
                if phase == 0:
                    ax = axes.flat[TRIALS.index(name)]
                    if mode == "Welded":
                        k = (pt >= -0.8) & (pt <= 0.1)
                        ax.scatter(pt[k], pd[k], s=7, color="black", label="Physical")
                    k = (st >= -0.8) & (st <= 0.1)
                    ax.scatter(st[k], sd[k], s=5, alpha=0.7, label=mode)
                    ax.set_title(name)
                    ax.set_yscale("symlog", linthresh=0.03)

            scores = Path(temp) / "scores.csv"
            replay(
                scores,
                feedforward=True,
                packet_observer=observe,
                wheel_inputs=None if mode == "Welded" else inp,
                welded_supports=mode == "Welded",
            )
            with scores.open() as r:
                for s in csv.DictReader(r):
                    name = s["trial"]
                    phase = int(s["phase_ms"])
                    w.writerow(
                        [
                            name,
                            phase,
                            mode,
                            s["physical_count"],
                            s["simulated_count"],
                            s["first_error_ms"],
                            *metrics[name, phase],
                        ]
                    )
    axes.flat[0].legend(fontsize=7)
    fig.supxlabel("Seconds from first high-pitch report")
    fig.supylabel("Acceleration change from baseline (g)")
    fig.savefig(HERE / "moving-treads.png", dpi=160)


if __name__ == "__main__":
    main()

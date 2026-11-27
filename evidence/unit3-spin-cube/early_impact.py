"""Inspect physical cube acceleration around each replay's first predicted mount tap."""

import csv
from tempfile import TemporaryDirectory
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import mount_replay as m
from actuated_mount import replay
from earlier_inputs import TRIALS
from tap_timing import events


def deviation(xyz, baseline):
    return np.linalg.norm(xyz - np.median(xyz[baseline], axis=0), axis=1)


def main():
    physical = m._load_structured("impact-samples.csv.gz")
    counters = m._load_structured("earlier-packets.csv.gz")
    fig, axes = plt.subplots(
        2, 4, figsize=(13, 6), layout="constrained", sharex=True, sharey=True
    )
    axes.flat[-1].set_visible(False)
    with (m.HERE / "early-impact.csv").open("w") as out, TemporaryDirectory() as tmp:
        writer = csv.writer(out, lineterminator="\n")
        writer.writerow(
            "trial phase_ms half_window_ms predicted_tap_s physical_first_tap_s physical_packets nearest_physical_ms peak_physical_g peak_sim_g".split()
        )

        def compare(name, phase, arr):
            pk = physical[physical["trial"] == name]
            pt = pk["packet_s"]
            pd = deviation(np.stack([pk[k] for k in "xyz"], axis=1) / 8192, pt < -4)
            ct = counters[counters["trial"] == name]
            physical_tap = events(ct["t_s"], ct["tap"])[0]
            st = arr[:, 0]
            sd = deviation(arr[:, 1:4], (st >= -2.9) & (st < -2.1))
            est = events(st, arr[:, 4])
            assert len(est), name
            first = est[0]
            for width in (0.05, 0.10):
                pw = abs(pt - first) <= width
                sw = abs(st - first) <= width
                writer.writerow(
                    [
                        name,
                        phase,
                        round(width * 1000),
                        first,
                        physical_tap,
                        len(set(pt[pw])),
                        1000 * min(abs(pt - first)),
                        max(pd[pw], default=""),
                        max(sd[sw]),
                    ]
                )
            if phase == 0:
                ax = axes.flat[TRIALS.index(name)]
                for t, d, color, label in [
                    (pt, pd, "black", "Physical"),
                    (st, sd, "teal", "Simulated"),
                ]:
                    k = (t >= -1) & (t <= 0.12)
                    ax.scatter(t[k], d[k], s=6, color=color, label=label)
                ax.axvline(first, color="teal", ls=":")
                ax.axvline(physical_tap, color="black", ls="--")
                ax.set_title(name)
                ax.set_yscale("symlog", linthresh=0.02)

        replay(Path(tmp) / "scores.csv", feedforward=True, packet_observer=compare)
    axes.flat[0].legend(fontsize=8)
    fig.supxlabel(
        "Seconds from first high-pitch report · lines show first tap packet in scoring window"
    )
    fig.supylabel("Acceleration change from resting baseline (g)")
    fig.savefig(m.HERE / "early-impact.png", dpi=160)


if __name__ == "__main__":
    main()

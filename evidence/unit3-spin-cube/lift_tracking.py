"""Isolate servo lag, then check whether gain-derived velocity compensation transfers."""

import csv
import tempfile
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from actuated_mount import replay
from earlier_inputs import HERE, TRIALS


def main():
    fig, axes = plt.subplots(
        2, 4, figsize=(13, 6), sharex=True, sharey=True, layout="constrained"
    )
    axes.flat[-1].set_visible(False)
    with (
        tempfile.TemporaryDirectory() as folder,
        (HERE / "lift-tracking.csv").open("w") as f,
    ):
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            "trial condition feedforward rmse_mm descent_crossing_error_ms count_mae first_tap_mae_ms".split()
        )
        for unloaded in (False, True):
            for ff in (False, True):
                paths = {}
                out = Path(folder) / "scores.csv"
                replay(out, ff, unloaded, paths)
                with out.open() as scores:
                    rows = list(csv.DictReader(scores))
                for ax, trial in zip(axes.flat, TRIALS):
                    ts, h, pred = paths[trial]
                    # h is the reference at the start of each 1 ms step.
                    expected = np.interp(ts, ts - 0.001, h)
                    win = (ts >= -0.8) & (ts <= 0.1)
                    rms = np.sqrt(np.mean((expected[win] - pred[win]) ** 2))

                    def crossing(z):
                        ii = np.flatnonzero((z[:-1] >= 60) & (z[1:] < 60) & win[1:])
                        if not len(ii):
                            return np.nan
                        i = ii[0]
                        return np.interp(60, z[i : i + 2][::-1], ts[i : i + 2][::-1])

                    err = 1000 * (crossing(pred) - crossing(expected))
                    r = [x for x in rows if x["trial"] == trial]
                    count = (
                        np.mean(
                            [
                                abs(
                                    int(x["physical_count"]) - int(x["simulated_count"])
                                )
                                for x in r
                            ]
                        )
                        if r
                        else ""
                    )
                    timing = (
                        np.mean(
                            [
                                abs(float(x["first_error_ms"]))
                                if x["first_error_ms"]
                                else 900
                                for x in r
                            ]
                        )
                        if r
                        else ""
                    )
                    condition = "unloaded" if unloaded else "loaded"
                    w.writerow([trial, condition, int(ff), rms, err, count, timing])
                    if not unloaded and not ff:
                        ax.plot(ts[win], expected[win], color="black", label="Recorded")
                    ax.plot(
                        ts[win],
                        pred[win],
                        ls="--" if unloaded else "-",
                        color="darkorange" if ff else "teal",
                        label=f"{condition}, " + ("compensated" if ff else "original"),
                        lw=1,
                    )
                    ax.set_title(trial)
    axes.flat[0].legend(fontsize=6)
    fig.supxlabel("Seconds from SDK pitch crossing")
    fig.supylabel("Lift height (mm)")
    fig.savefig(HERE / "lift-tracking.png", dpi=170)


if __name__ == "__main__":
    main()

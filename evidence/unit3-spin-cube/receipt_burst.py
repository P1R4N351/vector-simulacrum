"""Bracket W26's delayed SDK burst without changing physical radio packet times."""

import csv
from pathlib import Path
import tempfile
import matplotlib.pyplot as plt
import numpy as np
import mount_replay as m
from actuated_mount import replay


def main():
    path = m._load_structured("earlier-path.csv.gz")
    path = path[path["trial"] == "W26"]
    t = path["t_s"]
    dt = np.diff(t)
    cadence = np.median(dt[t[:-1] < -0.8])
    i = np.flatnonzero(dt > 3 * cadence).item()
    # Stop before the first ordinary-length gap after the cluster of short intervals.
    end = next(k for k in range(i + 2, len(dt)) if dt[k] >= 0.7 * cadence)
    regular = path.copy()
    regular["t_s"][i : end + 1] = np.linspace(t[i], t[end], end - i + 1)
    print(
        "cadence, bracketing times, inferred interval",
        cadence,
        t[i],
        t[end],
        (t[end] - t[i]) / (end - i),
    )
    with (m.HERE / "receipt-burst-clock.csv").open("w") as out:
        w = csv.writer(out, lineterminator="\n")
        w.writerow("receipt_s reconstructed_s lift_mm".split())
        w.writerows(zip(t, regular["t_s"], path["lift_mm"]))
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), sharey=True, layout="constrained")
    with (
        tempfile.TemporaryDirectory() as folder,
        (m.HERE / "receipt-burst.csv").open("w") as out,
    ):
        w = csv.writer(out, lineterminator="\n")
        w.writerow(
            "clock condition height_rmse_mm mean_count_error mean_first_tap_error_ms".split()
        )
        for ax, (clock, p) in zip(
            axes, [("Receipt", path), ("Reconstructed", regular)]
        ):
            sub = (p["t_s"] >= -0.8) & (p["t_s"] <= 0.1)
            ax.plot(
                p["t_s"][sub],
                p["lift_mm"][sub],
                ".-",
                color="black",
                label="SDK samples",
            )
            for unloaded in (True, False):
                tracks = {}
                file = Path(folder) / "scores.csv"
                replay(file, True, unloaded, tracks, path_override=p, trials=["W26"])
                ts, _, pred = tracks["W26"]
                win = (ts >= -0.8) & (ts <= 0.1)
                expected = np.interp(ts, p["t_s"], p["lift_mm"])
                rms = np.sqrt(np.mean((expected[win] - pred[win]) ** 2))
                with file.open() as src:
                    r = list(csv.DictReader(src))
                counts = [
                    abs(int(x["physical_count"]) - int(x["simulated_count"])) for x in r
                ]
                errors = [
                    abs(float(x["first_error_ms"])) if x["first_error_ms"] else 900
                    for x in r
                ]
                condition = "Unloaded" if unloaded else "Loaded"
                w.writerow(
                    [
                        clock,
                        condition,
                        rms,
                        np.mean(counts) if r else "",
                        np.mean(errors) if r else "",
                    ]
                )
                ax.plot(ts[win], pred[win], label=condition)
            ax.set(title=clock, xlabel="Seconds from first high-pitch report")
            ax.legend(fontsize=8)
    axes[0].set_ylabel("Lift height (mm)")
    fig.savefig(m.HERE / "receipt-burst.png", dpi=160)


if __name__ == "__main__":
    main()

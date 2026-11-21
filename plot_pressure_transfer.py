"""Check fixed replay models on SDK recordings outside the pressure sweep."""

import csv
import json

import matplotlib.pyplot as plt
import numpy as np

from replay_endurance_drift import relative_center
from replay_measured_turn import run
from replay_native_stops import DATA as NATIVE
from replay_native_stops import delayed
from replay_pivot import DATA, ROOT, configure_round


def main():
    fig, axes = plt.subplots(
        2, 5, figsize=(15, 6), sharex="row", sharey="row", layout="constrained"
    )
    writer_rows = []
    index = 0
    for filename in ("unit2-ccw100.jsonl", "unit2-repeat-stops.jsonl"):
        events = list(map(json.loads, (NATIVE / filename).read_text().splitlines()))
        for cmd in [e for e in events if e.get("name") == "wheels"]:
            stop = next(
                e["issued_t"]
                for e in events
                if e.get("why") == "segment_end" and e["t"] > cmd["t"]
            )
            rows = [
                e
                for e in events
                if e["k"] == "s" and cmd["t"] - 0.4 <= e["t"] <= stop + 0.65
            ]
            t = np.array([r["t"] - stop for r in rows])
            w = np.array([[r["wl"], r["wr"]] for r in rows])
            gyro = np.degrees([r["gyro"][2] for r in rows])
            ref = relative_center(
                np.array([r["pose"][:2] for r in rows]),
                np.array([r["yaw"] for r in rows]),
            )
            ax, xy = axes[:, index]
            ax.plot(t, gyro, ".-", ms=3, lw=1, color="#233549", label="Hardware")
            xy.plot(*ref.T, color="#233549", label="SDK odometry")
            for label, step, conf, color in (
                ("Current", 1, None, "#b75d43"),
                ("Round 0.25 ms", 0.25, configure_round, "#1b8174"),
                ("Round 0.125 ms", 0.125, configure_round, "#745ba5"),
            ):
                clock, o = run(
                    rows,
                    t,
                    w,
                    42,
                    dense=True,
                    include_pose=True,
                    configure=conf,
                    step_ms=step,
                )
                pred = delayed(clock, o[:, 0], t, 17)
                path = relative_center(o[:, 3:5], o[:, 5])
                ax.plot(t, pred, color=color, lw=1.3, label=label)
                xy.plot(*path.T, color=color, lw=1, label=label)
                xy.scatter(*path[-1], s=15, color=color)
                end_error = np.linalg.norm(path[-1] - ref[-1]) if t[-1] > 0.25 else None
                for window, lo, hi in (("steady", -0.65, -0.10), ("stop", -0.10, 0.25)):
                    mask = (t >= lo) & (t <= hi)
                    writer_rows.append(
                        (
                            filename,
                            cmd["label"],
                            label,
                            window,
                            mask.sum(),
                            np.sqrt(np.mean((pred[mask] - gyro[mask]) ** 2))
                            if mask.any()
                            else None,
                            end_error,
                        )
                    )
            if t[-1] < 0:
                ax.text(
                    0.03,
                    0.94,
                    "Stop missing",
                    va="top",
                    transform=ax.transAxes,
                    fontsize=8,
                )
            ax.set_title(cmd["label"].replace("_", " "))
            ax.axvline(0, color="0.7", lw=0.8)
            ax.set_xlabel("Time from stop (s)")
            xy.set_xlabel("Forward (mm)")
            xy.set_aspect("equal", adjustable="box")
            index += 1
    for ax in axes.flat:
        ax.grid(alpha=0.18)
    axes[0, 0].set_ylabel("Head gyro Z (°/s)")
    axes[1, 0].set_ylabel("Drive center left (mm)")
    axes[0, -1].legend(fontsize=7)
    fig.suptitle("Transfer to two separate SDK recordings · 100 mm/s")
    fig.savefig(ROOT / "evidence/physics-plots/pressure-transfer.png", dpi=150)
    with (DATA / "pressure-transfer.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            (
                "session",
                "turn",
                "model",
                "window",
                "samples",
                "gyro_rmse_dps",
                "final_vector_error_mm",
            )
        )
        w.writerows(writer_rows)


if __name__ == "__main__":
    main()

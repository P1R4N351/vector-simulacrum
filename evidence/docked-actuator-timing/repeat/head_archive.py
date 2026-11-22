"""Challenge the frozen receipt shift on the two earlier head sweeps."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2]))
from diagnose_head_gyro import GyroReplay
from diagnose_lift_timing import maneuvers
from simulacrum import replay


def main():
    with (HERE / "head-transfer.csv").open() as stream:
        shift = float(next(csv.DictReader(stream))["shift_ms"]) / 1000
    values = replay.nominal_values()
    results = []
    for session, seg, states in maneuvers(values, "head"):
        initial = replay._state_at(states, seg.t_cmd)
        sim = GyroReplay(values=dict(values, **replay.OPEN_TABLE), head_deg=np.degrees(initial["head"]),
                         lift_mm=initial["lift"], noise_seed=0)
        pre = .32
        outputs = replay._run(sim, seg, pre)
        ts = (1 + np.arange(len(outputs))) * sim.dt - pre
        samples = [s for s in states if -.3 <= s["t"] - seg.t_cmd <= min(1, seg.t_stop-seg.t_cmd)]
        t = np.array([s["t"] - seg.t_cmd for s in samples])
        angles = np.degrees([s["head"] for s in samples])
        start = np.median(angles[t < 0])
        if abs(seg.target - start) < 3:
            continue
        gyro = -np.degrees([s["gyro"][1] for s in samples])
        gyro -= np.median(gyro[t < 0])
        predicted = np.array([s["pitch_speed"] for s in outputs])
        predicted -= np.median(predicted[ts < 0])
        row = dict(session=session, move=seg.label, initial_deg=start, target_deg=seg.target)
        valid = t >= 0
        for key, measured, modeled in [("deg", angles, [s["head_deg"] for s in outputs]),
                                        ("dps", gyro, predicted)]:
            for name, delay in [("baseline", 0), ("shifted", shift)]:
                residual = np.interp(t[valid] - delay, ts, modeled) - measured[valid]
                row[f"{name}_{key}"] = np.sqrt(np.mean(residual ** 2))
        results.append(row)
    with (HERE / "head-archive.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=results[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(results)
    fig, axes = plt.subplots(1, 2, figsize=(9, 5), layout="constrained")
    for ax, key, label in zip(axes, ("deg", "dps"), ("Angle RMS error (°)", "Pitch gyro RMS error (°/s)")):
        for row in results:
            color = "#97500b" if row["session"] == "Nov15" else "#285e97"
            ax.plot([0, 1], [row["baseline_"+key], row["shifted_"+key]], "o-", color=color, alpha=.78)
        ax.set(xticks=[0, 1], xticklabels=["Frozen", "+92 ms"], ylabel=label, xlim=(-.15, 1.15), ylim=(0,None))
        ax.grid(axis="y", alpha=.2)
    fig.suptitle("Earlier physical head sweeps · one line per stroke")
    fig.legend(handles=[plt.Line2D([], [], color=c, label=s) for s,c in
                        [("Nov 15", "#97500b"), ("Nov 18", "#285e97")]], loc="outside lower center", ncol=2)
    fig.savefig(HERE / "head-archive.png", dpi=160)


if __name__ == "__main__":
    main()

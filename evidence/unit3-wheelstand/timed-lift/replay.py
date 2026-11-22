"""Run with PYTHONPATH=. from the repository root."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from simulacrum import params
from simulacrum.robot import VectorSim

HERE = Path(__file__).resolve().parent
DURATIONS = {"fast": 0.6, "matched": 0.8, "slow": 1.4}


def crossing(t, p, level):
    i = np.flatnonzero(p >= level)[0]
    return np.interp(level, p[i-1:i+1], t[i-1:i+1])


def replay(values, direction, duration):
    ends = [values["lift_height_min_mm"], values["lift_height_max_mm"]]
    if direction == "lower":
        ends.reverse()
    sim = VectorSim(values, lift_mm=ends[0])
    sim.advance(0.5)
    sim.command_lift_height(ends[1], duration_s=duration)
    h = np.array([sim.step().lift_height_mm for _ in range(round(2.5/sim.dt))])
    return np.arange(1, len(h)+1)*sim.dt, (h-h[0])/(h[-1]-h[0])


def rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def main():
    values = params.values(params.load(model="vector_2"))
    fig, axes = plt.subplots(2, 3, figsize=(12, 6), sharex=True, sharey=True, layout="constrained")
    results = []
    for col, condition in enumerate(DURATIONS):
        folder = HERE.parent / ("D2" if condition == "matched" else "D1")
        sensors = np.genfromtxt(folder/"state.csv.gz", delimiter=",", names=True)
        events = {(e["action"], e["kind"]):float(e["sec"]) for e in rows(folder/"actions.csv")}
        for row, direction in enumerate(("raise", "lower")):
            ax = axes[row, col]
            predictions = []
            for duration, label, color in [(0, "Previous", "#ad5822"), (DURATIONS[condition], "Timed", "#006e73")]:
                t, p = replay(values, direction, duration)
                x = t-crossing(t, p, .1)
                predictions.append(crossing(x, p, .9))
                ax.plot(x, p, label=label, color=color, zorder=4, linestyle="--" if duration==0 else "-")
            for r in rows(folder/"moves.csv"):
                if (r["condition"], r["direction"], r["timing_valid"]) != (condition, direction, "1"):
                    continue
                name = " ".join((condition, direction, r["repeat"]))
                begin, end = events[name, "begin"], events[name, "end"]
                select = (sensors["sec"] >= begin-.5) & (sensors["sec"] <= end+.7)
                t = sensors["sec"][select]-begin
                h = sensors["lift"][select]
                p = (h-np.median(h[t<0]))/float(r["travel_mm"])
                ax.plot(t-crossing(t, p, .1), p, color="#495889", alpha=.65,
                        label="Encoder" if r["repeat"]=="1" else None)
                results.append([folder.name, name, float(r["rise_10_90_s"]), *predictions])
            ax.set_title(f"{direction.capitalize()} · {DURATIONS[condition]} s · {folder.name}")
            ax.set_xlim(-.23, 1.35)
    axes[0,0].legend()
    fig.supxlabel("Seconds from 10% travel")
    fig.supylabel("Fraction of encoder travel")
    fig.savefig(HERE/"profiles.png", dpi=150)
    with (HERE/"crossings.csv").open("w") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["recording", "stroke", "physical_s", "previous_s", "timed_s"])
        writer.writerows(results)


if __name__ == "__main__":
    main()

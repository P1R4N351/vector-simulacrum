"""Check a command-to-encoder time shift on second repetitions."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent


def rows(name):
    with (here / name).open() as f:
        return list(csv.DictReader(f))


raw = rows("recording.csv")
states = [r for r in raw if r["event"] == "state"]
commands = [r for r in raw if r["event"] == "command"]
model = rows("replay.csv")
time = np.array([float(r["s"]) for r in states])
assert np.array_equal(time, [float(r["s"]) for r in model])
traces = [np.array([float(r["lift_mm"]) for r in source]) for source in (states, model)]


def crossing(t, fraction, level):
    i = np.flatnonzero((fraction[:-1] < level) & (fraction[1:] >= level))[0]
    return np.interp(level, fraction[i:i+2], t[i:i+2])


records = []
fig, axes = plt.subplots(2, 2, figsize=(9, 6), layout="constrained", sharex=True, sharey=True)
for i in range(1, len(commands), 2):
    cue = float(commands[i]["s"])
    duration = float(commands[i]["duration_s"])
    height = int(commands[i-1]["target_mm"])
    times = time - cue
    use = (times >= -.2) & (times <= 1.1)
    t = times[use]
    fractions = []
    for trace in traces:
        start = np.median(trace[(times >= -.2) & (times < 0)])
        fractions.append((start - trace[use]) / (start - 32))
    observed, predicted = fractions
    delta = [crossing(t, predicted, level) - crossing(t, observed, level) for level in (.2, .5, .8)]
    records.append((duration, delta))
    if i < 8:
        continue
    shift = np.median([d[1] for dur, d in records[:4] if dur == duration])
    ax = axes[int(height == 62), int(duration == .6)]
    for x, y, style, label in [(t, observed, "o-", "Robot"), (t, predicted, ".--", "Canonical"),
                               (t-shift, predicted, ":", "Shift estimated on first repetitions")]:
        ax.plot(x, y, style, ms=3, label=label)
    ax.set_title(f"{height} → 32 mm · {duration*1000:.0f} ms request")
    ax.grid(alpha=.2)
    print(f"Trial {(i+1)//2}, shift {1000*shift:.0f} ms, model minus robot at 20/50/80% "
          f"{np.round(1000*np.array(delta)).astype(int)}, shifted "
          f"{np.round(1000*(np.array(delta)-shift)).astype(int)} ms")
for ax in axes[-1]:
    ax.set_xlabel("Seconds after command")
for ax in axes[:, 0]:
    ax.set_ylabel("Fraction of start-to-target travel")
axes[0,0].legend(fontsize=8)
axes[0,0].set_xlim(-.12,.85)
fig.savefig(here / "phase.png", dpi=160)

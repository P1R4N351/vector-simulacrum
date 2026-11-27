"""Test whether one command-to-encoder timing offset transfers between robots."""
import csv
import gzip
import json
from math import degrees
from pathlib import Path
from statistics import median

import matplotlib.pyplot as plt
from simulacrum import params
from simulacrum.robot import VectorSim

ROOT = Path(__file__).resolve().parent
FRACTIONS = (.10, .50, .90)
RECORDINGS = [
    ("Unit 1", "repeat/unit1.jsonl", "vector_1"),
    ("Unit 2", "unit2.jsonl", "vector_1"),
    ("Unit 3", "repeat/unit3.jsonl", "vector_2"),
    ("Unit 3 timed", "../unit3-lift-history/physical.jsonl.gz", "vector_2"),
    ("Workshop repeat", "workshop-validation.jsonl", "vector_1"),
]


def crossing(trace, threshold, direction):
    for (t0, h0), (t1, h1) in zip(trace, trace[1:]):
        if direction * (h0 - threshold) < 0 <= direction * (h1 - threshold):
            return t0, t0 + (threshold - h0) * (t1 - t0) / (h1 - h0), t1
    return None


def replay(label, path, model):
    opener = gzip.open if path.endswith(".gz") else open
    with opener(ROOT / path, "rt") as stream:
        events = [json.loads(line) for line in stream]
    states = [r for r in events if r.get("kind") == "state" or r.get("k") == "s"]
    cues = [r for r in events if r.get("name") == "lift" or r.get("kind") == "command"]
    values = params.values(params.load(model=model))
    for i, cue in enumerate(cues):
        t = cue["t"]
        start = next(r for r in reversed(states) if r["t"] < t)
        h = start["lift"]
        target = 32 + 60 * (cue["norm"] if "norm" in cue else cue["pos"])
        if abs(target - h) < 2:
            continue
        limit = min(t + 2, cues[i+1]["t"]) if i+1 < len(cues) else t+2
        measured = [(r["t"] - t, r["lift"]) for r in [start, *(r for r in states if t <= r["t"] < limit)]]
        sim = VectorSim(values, head_deg=degrees(start["head"]), lift_mm=h, charger_contact=True)
        sim.command_lift_height(target, duration_s=.7 if "pos" in cue else 0)
        simulated = [(0, sim.lift_height_mm())]
        while simulated[-1][0] < measured[-1][0]:
            sim.step()
            simulated.append((sim.tick * sim.dt, sim.lift_height_mm()))
        direction = 1 if target > h else -1
        for frac in FRACTIONS:
            threshold = h + (target-h)*frac
            a = crossing(measured, threshold, direction)
            b = crossing(simulated, threshold, direction)
            yield [label, i+1, "Raise" if direction==1 else "Lower", target, frac,
                   *(a or (None, None, None)), b[1] if b else None]


def main():
    rows = [r for source in RECORDINGS for r in replay(*source)]
    usable = [r for r in rows if r[6] is not None and r[8] is not None]
    delay = median(r[6]-r[8] for r in usable if r[0]=="Unit 1" and r[4]==.1)
    with (ROOT/"lift-crossings.csv").open("w") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow("recording command direction target_mm fraction last_before_s interpolated_s first_after_s sim_s".split())
        writer.writerows(rows)
    fig, axes = plt.subplots(2, 5, figsize=(13,5.4), sharex=True, sharey=True, layout="constrained")
    print("Frozen delay (ms)", round(delay*1000))
    for col,(label,_,_) in enumerate(RECORDINGS):
        group = [r for r in usable if r[0]==label]
        for row,direction in enumerate(("Raise","Lower")):
            ax = axes[row,col]
            strokes = {}
            for r in group:
                if r[2]==direction:
                    strokes.setdefault(r[1], []).append(r)
            spans = []
            for trace in strokes.values():
                ax.plot([100*r[4] for r in trace], [1000*(r[6]-r[8]) for r in trace], "o-", alpha=.55, ms=3)
                if len(trace)==3:
                    spans.append(1000*((trace[2][6]-trace[0][6])-(trace[2][8]-trace[0][8])))
            ax.axhline(0,color=".6",ls=":")
            ax.axhline(delay*1000,color="black",ls="--",lw=.9)
            if row==0: ax.set_title(label)
            if col==0: ax.set_ylabel(f"{direction}\nMeasured − sim (ms)")
            if row==1: ax.set_xlabel("Stroke (%)")
            ax.set_xticks([10,50,90])
            print(label,direction,"complete spans",len(spans),"median span residual",round(median(spans)) if spans else None)
        print(label,"crossings",len(group),"MAE(median) ms",
              *[round(1000*median(abs(r[6]-r[8]-lag) for r in group)) for lag in (0,delay)])
    fig.savefig(ROOT/"lift-crossings.png",dpi=150)


if __name__=="__main__":
    main()

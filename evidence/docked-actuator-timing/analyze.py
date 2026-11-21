"""Replay the matched full strokes and measure timing at the recorded SDK cadence."""
import csv
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
from diagnose_head_gyro import GyroReplay, onset
from simulacrum import params, recording, replay

FILES = {"K9N7": HERE / "k9n7.jsonl", "C7K8": HERE.parent / "hw-rec-20261118/run-a-rest-headlift.jsonl",
         "P2Y8": HERE / "p2y8.jsonl"}
COLORS = {"K9N7": "#b64425", "C7K8": "#2171a5", "P2Y8": "#258557", "Frozen": "#222222"}


def crossing(t, progress, threshold):
    i = np.flatnonzero((t >= 0) & (progress >= threshold))[0]
    return np.interp(threshold, progress[i-1:i+1], t[i-1:i+1])


def measures(t, position, speed, joint, sign):
    base = np.median(position[t < 0])
    travel = sign * (position - base)
    span = np.median(travel[t > 1.25])
    if joint == "head":
        lo, hi, start, _ = onset(t, speed, sign)
    else:
        # First sustained encoder displacement (well below one full mm).
        over = travel > 0.15
        i = np.flatnonzero((t[:-1] >= 0) & over[:-1] & over[1:])[0]
        lo, hi = t[i-1:i+1]
        start = (max(0, lo) + hi) / 2
    return start, hi-lo, span, crossing(t, travel, .9*span)-crossing(t, travel, .1*span)


def main():
    fig, axes = plt.subplots(2, 2, figsize=(10, 7), sharex=True, layout="constrained")
    scores = []
    for bot, path in FILES.items():
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        states = [r for r in rows if r["k"] == "s"]
        values = params.values(params.load(model="vector_2" if bot == "P2Y8" else "vector_1"))
        previous = {}
        for c in rows:
            joint = c.get("name")
            if joint not in ("head", "lift"):
                continue
            target = c["deg"] if joint == "head" else 32 + 60*c["norm"]
            ends = (-22, 45) if joint == "head" else (32, 92)
            last = previous.get(joint)
            previous[joint] = target
            if {last, target} != set(ends):
                continue
            sign = np.sign(target-last)
            samples = [r for r in states if -.30 <= r["t"]-c["t"] <= 1.53]
            t = np.array([r["t"]-c["t"] for r in samples])
            actual = np.array([r[joint] for r in samples])
            speed = -np.degrees([r["gyro"][1] for r in samples])
            if joint == "head":
                actual = np.degrees(actual)
            # A commanded endpoint can be lost between actions. Match measured endpoints.
            if abs(np.median(actual[t < 0])-last) > 5 or abs(np.median(actual[t > 1.25])-target) > 5:
                print("Unmatched endpoints:", bot, joint, c["t"])
                continue
            initial = replay._state_at(samples, c["t"])
            sim = GyroReplay(values=dict(values, **replay.OPEN_TABLE), head_deg=np.degrees(initial["head"]),
                             lift_mm=initial["lift"], noise_seed=0)
            seg = recording.Segment(joint, joint, 0, 1.55, target=target)
            series = replay._run(sim, seg, .32)
            modeled = [series[int(round((u+.32)/sim.dt))-1] for u in t]
            axis = axes[int(joint == "lift"), int(sign < 0)]
            for source, pos, vel in [(bot, actual, speed), ("Frozen", np.array([
                r["head_deg" if joint == "head" else "lift_mm"] for r in modeled]),
                np.array([r["pitch_speed"] for r in modeled]))]:
                start, width, span, duration = measures(t, pos, vel, joint, sign)
                scores.append(dict(bot=bot, joint=joint, direction="up" if sign>0 else "down",
                                   command_s=c["t"], source=source, onset_ms=1000*start,
                                   onset_bracket_ms=1000*width, travel=span, t10_90_ms=1000*duration))
                axis.plot(t-start, pos, color=COLORS[source], alpha=.52, lw=1.4,
                          ls="--" if source=="Frozen" else "-")
            axis.set(title=joint.capitalize()+(" raise" if sign>0 else " lower"),
                     ylabel="Head angle (°)" if joint=="head" else "Lift height (mm)", xlim=(-.14,1))
    for ax in axes.flat:
        ax.grid(alpha=.2)
        ax.axvline(0, color=".6", lw=.7)
    for ax in axes[-1]:
        ax.set_xlabel("Seconds from first motion")
    fig.suptitle("Full actuator strokes aligned at first motion")
    fig.legend(handles=[plt.Line2D([], [], color=v, label=k, ls="--" if k=="Frozen" else "-")
                        for k,v in COLORS.items()], loc="outside lower center", ncol=4)
    fig.savefig(HERE / "profiles.png", dpi=180)
    with (HERE / "moves.csv").open("w") as f:
        writer=csv.DictWriter(f,fieldnames=scores[0])
        writer.writeheader()
        writer.writerows(scores)
    for bot in FILES:
        for joint in ["head","lift"]:
            for d in ["up","down"]:
                print(bot,joint,d, end=" ")
                for src in (bot,"Frozen"):
                    rr=[r for r in scores if (r["bot"],r["joint"],r["direction"],r["source"])==(bot,joint,d,src)]
                    print(len(rr), [round(np.median([r[k] for r in rr]),1) for k in
                                    ["onset_ms","t10_90_ms","travel","onset_bracket_ms"]],end=" ")
                print()

if __name__=="__main__":
    main()

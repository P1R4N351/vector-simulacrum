"""Summarize one physical recording, retaining the host sample brackets for transitions."""
import csv
import json
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze import GyroReplay, params, recording, replay

path = Path(sys.argv[1])
records = [json.loads(line) for line in path.read_text().splitlines()]
model_name = sys.argv[2]
values = dict(params.values(params.load(model=model_name)), **replay.OPEN_TABLE)
commands = [r for r in records if r["k"] == "command"]
states = [r for r in records if r["k"] == "s"]
assert len(commands) == 12 and all(r["code"] == 0 for r in records if r["k"] == "done")
fig, axes = plt.subplots(2, 4, figsize=(13, 5), sharex=True, layout="constrained")
scores = []
for i, cmd in enumerate(commands):
    near = [r for r in states if -.20 < r["t"]-cmd["t"] < 1.32]
    t = np.array([r["t"]-cmd["t"] for r in near])
    gyro = -np.degrees([r["gyro"][1] for r in near])
    angle = np.degrees([r["head"] for r in near])
    active = np.flatnonzero((t >= 0) & (abs(gyro) > 10))
    first, last = active[0], active[-1]
    assert first > 0 and last+1 < len(t)
    settled = np.array([bool(r["status"] & 0x200) for r in near])
    rises = np.flatnonzero((t[1:] > t[first]) & settled[1:] & ~settled[:-1])
    edge = rises.item()
    row = dict(cycle=cmd["cycle"], target=cmd["target"], initial_deg=np.median(angle[t<0]),
               onset_lo=t[first-1], onset_hi=t[first], clear_lo=t[last], clear_hi=t[last+1],
               settled_lo=t[edge], settled_hi=t[edge+1], peak_dps=max(abs(gyro)))
    init = replay._state_at(near, cmd["t"])
    sim = GyroReplay(values=values, head_deg=np.degrees(init["head"]), lift_mm=init["lift"], noise_seed=0)
    segment = recording.Segment("head","head",0,1.35,target=cmd["target"])
    trace = replay._run(sim,segment,.32)
    ts = (np.arange(len(trace))+1)*sim.dt-.32
    modeled = [np.array([r[key] for r in trace]) for key in ("head_deg","pitch_speed")]
    window = (t >= 0) & (t <= 1)
    for measured, signal, name in zip([angle, gyro], modeled, ["angle","gyro"]):
        for shift in (0,.0916):
            error = np.interp(t[window]-shift,ts,signal)-measured[window]
            row[f"{name}_rms_{round(1000*shift)}ms"] = np.sqrt(np.mean(error**2))
    scores.append(row)
    color = ["#137b80","#be691d","#6c4b9c"][cmd["cycle"]-1]
    for ax, y, signal in zip(axes[:,i%4], [angle, gyro], modeled):
        ax.plot(ts+.0916,signal,"--",color=color,lw=.9,alpha=.6)
        ax.plot(t,y,".-",color=color,ms=3,alpha=.8,label=f"Cycle {cmd['cycle']}")
    axes[0,i%4].set_title(f"Target {cmd['target']}°")
    axes[1,i%4].plot(t[edge:edge+2], [0,0], "|-",color=color,lw=3)
axes[0,0].set_ylabel("Encoder angle (°)")
axes[1,0].set_ylabel("Pitch gyro (°/s)")
for ax in axes[-1]: ax.set_xlabel("Seconds from command")
fig.legend(*axes[0,0].get_legend_handles_labels(),loc="outside lower center",ncol=3)
fig.suptitle(f"{path.stem.upper()} docked head sequence · bars mark head-in-position transition")
fig.savefig(path.with_suffix(".png"),dpi=150)
with path.with_suffix(".csv").open("w") as stream:
    writer = csv.DictWriter(stream,fieldnames=scores[0],lineterminator="\n")
    writer.writeheader()
    writer.writerows(scores)
print(len(states), "samples;", np.median(np.diff([s["t"] for s in states]))*1000,"ms")
for target in (40,-18,15,-8):
    group = [r for r in scores if r["target"] == target]
    print(target,"peak gyro",np.round([r["peak_dps"] for r in group]),"span",np.round([1000*(r["clear_lo"]+r["clear_hi"]-r["onset_lo"]-r["onset_hi"])/2 for r in group]))

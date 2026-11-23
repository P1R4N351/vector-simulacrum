"""Replay accepted docked strokes and compare the vision rate gate at SDK receipt times."""
import csv
import json
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
timing = here.parents[1] / "docked-actuator-timing"
sys.path.insert(0, str(timing))
from analyze import GyroReplay, params, recording, replay

sources = {"Unit 1": timing/"repeat/unit1.jsonl", "Unit 2": timing/"unit2.jsonl",
           "Unit 3": timing/"repeat/unit3.jsonl"}
with (timing/"repeat/moves.csv").open() as stream:
    moves = [r for r in csv.DictReader(stream) if r["joint"] == "head" and r["source"] == r["bot"]]

def edges(t, mask):
    active = np.flatnonzero((t >= 0) & mask)
    if not len(active) or active[0] == 0 or active[-1]+1 == len(t):
        raise ValueError("Unbracketed gyro transition")
    first, last = active[0], active[-1]
    return (t[first-1]+t[first])/2, (t[last]+t[last+1])/2

fig, axes = plt.subplots(3, 2, figsize=(10, 8), sharex=True, sharey=True, layout="constrained")
rows = []
for j, (bot, source) in enumerate(sources.items()):
    states = [s for s in map(json.loads, source.read_text().splitlines()) if s["k"] == "s"]
    v = params.values(params.load(model="vector_2" if bot == "Unit 3" else "vector_1"))
    for move in (m for m in moves if m["bot"] == bot):
        command = float(move["command_s"])
        near = [s for s in states if -.30 <= s["t"]-command <= 1.53]
        t = np.array([s["t"]-command for s in near])
        target = 45 if move["direction"] == "up" else -22
        initial = replay._state_at(near, command)
        sim = GyroReplay(values=dict(v, **replay.OPEN_TABLE), head_deg=np.degrees(initial["head"]),
                         lift_mm=initial["lift"], noise_seed=0)
        seg = recording.Segment("head","head",0,1.55,target=target)
        tr = replay._run(sim, seg, .32)
        ts = (np.arange(len(tr))+1)*sim.dt-.32
        predicted = np.abs(np.interp(t, ts, [r["pitch_speed"] for r in tr]))
        measured = np.abs(np.degrees([s["gyro"][1] for s in near]))
        onset, clear = edges(t, measured > 10)
        m_onset, m_clear = edges(t, predicted > 10)
        # Current-state half of BlockWorld's motion check. Exclude pre-stroke activity.
        moving = np.array([not (s["status"] & 0x200) or bool(s["status"] & 0x8000) for s in near])
        release = np.flatnonzero((t > onset) & ~moving & (np.arange(len(t)) > 0) & np.r_[False, moving[:-1]])
        last_moving = t[release[0]-1] if len(release) else np.nan
        release = t[release[0]] if len(release) else np.nan
        rows.append(dict(bot=bot, direction=move["direction"], command_s=command,
                         physical_start_ms=1000*onset, physical_clear_ms=1000*clear,
                         sim_start_ms=1000*m_onset, sim_clear_ms=1000*m_clear,
                         status_last_moving_sample_ms=1000*last_moving, status_first_clear_sample_ms=1000*release,
                         wheels_moving_samples=sum(bool(s["status"] & 0x8000) for s in near), sample_spacing_ms=1000*np.median(np.diff(t))))
        ax = axes[j, int(target < 0)]
        ax.plot(t, measured, ".-", color="#176870", alpha=.62)
        ax.plot(t, predicted, "--", color="#a85d22", alpha=.65)
        if np.isfinite(release): ax.plot(release, 10, "v", color="#54237d")
        ax.set_title(f"{bot} · {move['direction']}")
for ax in axes.flat:
    ax.axhline(10,color=".4",lw=.8)
    ax.set_xlim(0,1.25)
for ax in axes[-1]: ax.set_xlabel("Seconds after host command")
for ax in axes[:,0]: ax.set_ylabel("Pitch gyro magnitude (°/s)")
fig.suptitle("Docked full head strokes · measured (teal), replay (orange)\nTriangles mark the first SDK camera-moving flag release")
fig.savefig(here/"profiles.png", dpi=145)
with (here/"moves.csv").open("w") as f:
    writer = csv.DictWriter(f, fieldnames=rows[0], lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
for bot in sources:
    r=[v for v in rows if v["bot"] == bot]
    for v in r:
        print(bot,v["direction"],*(round(v[k]) if np.isfinite(v[k]) else "-" for k in
              ("physical_start_ms","physical_clear_ms","sim_start_ms","sim_clear_ms","status_first_clear_sample_ms","sample_spacing_ms")))

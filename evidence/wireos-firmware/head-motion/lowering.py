"""Challenge a faster Vector 2 lowering profile on independent full and partial strokes."""
import csv
import json
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
timing = here.parents[1] / "docked-actuator-timing"
sys.path.insert(0, str(timing))
from analyze import GyroReplay, params, replay, recording


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


# An acceleration-limited triangular move takes time proportional to 1/sqrt(a).
fit = [r for r in rows(here/"unit3-aligned.csv") if r["setting"] == "Docked" and r["direction"] == "down"]
scale = np.median([(float(r["modeled_above_threshold_ms"])/float(r["observed_above_threshold_ms"]))**2 for r in fit])
values = dict(params.values(params.load(model="vector_2")), **replay.OPEN_TABLE)
shift = float(rows(timing/"repeat/head-transfer.csv")[0]["shift_ms"]) / 1000
full = [s for s in map(json.loads, (timing/"unit3.jsonl").read_text().splitlines()) if s["k"] == "s"]
partial = [dict(t=float(s["receipt_s"]), head=float(s["head_rad"]), lift=float(s["lift_mm"]), gyro=[0,float(s["gyro_y"]),0])
           for s in rows(timing/"partial-strokes/states.csv")]
cases = [("Off dock", float(r["command_s"]), -22, full) for r in rows(here/"unit3-aligned.csv")
         if r["setting"] == "Off dock" and r["direction"] == "down"]
for c in rows(timing/"partial-strokes/commands.csv"):
    start, target = float(c["start_s"]), float(c["target"])
    if c["joint"] == "head" and target < np.degrees(replay._state_at(partial, start)["head"]):
        cases.append(("Partial", start, target, partial))

fig, axes = plt.subplots(len(cases), 2, figsize=(9, 2*len(cases)), sharex=True, layout="constrained")
scores = []
for panels, (group, start, target, states) in zip(axes, cases):
    initial = replay._state_at(states, start)
    near = [s for s in states if 0 <= s["t"]-start <= 1]
    t = np.array([s["t"]-start for s in near])
    measures = [np.degrees([s["head"] for s in near]), -np.degrees([s["gyro"][1] for s in near])]
    row = dict(group=group, command_s=start, initial_deg=np.degrees(initial["head"]), target_deg=target, accel_scale=scale)
    for panel, signal in zip(panels, measures):
        panel.plot(t, signal, "k.", label="Physical")
    for key, gain, color in [("frozen", 1, "#28758a"), ("faster", scale, "#be6825")]:
        model = dict(values, head_accel_default_radps2=values["head_accel_default_radps2"]*gain)
        sim = GyroReplay(values=model, head_deg=row["initial_deg"], lift_mm=initial["lift"], noise_seed=0)
        path = replay._run(sim, recording.Segment("head","head",0,1.05,target=target), .32)
        ts = (1+np.arange(len(path)))*sim.dt-.32
        # Compare angular speed at measured positions across the middle 80% of travel.
        # Restrict the lookup to the descending pass to exclude settling oscillations.
        position = np.array([s["head_deg"] for s in path])
        progress = (position-row["initial_deg"])/(target-row["initial_deg"])
        physical_progress = (measures[0]-row["initial_deg"])/(target-row["initial_deg"])
        middle = (physical_progress >= .1) & (physical_progress <= .9)
        indices = np.flatnonzero((ts >= 0) & (progress >= .02) & (progress <= .98))
        assert middle.any() and np.all(np.diff(progress[indices]) >= 0)
        levels, bins = np.unique(progress[indices], return_inverse=True)
        speed = np.array([s["pitch_speed"] for s in path])
        level_speed = np.bincount(bins, weights=speed[indices]) / np.bincount(bins)
        expected = np.interp(physical_progress[middle], levels, level_speed)
        row[f"{key}_angle_matched_dps"] = np.sqrt(np.mean((expected-measures[1][middle])**2))
        row["middle_samples"] = int(middle.sum())
        for panel, measured, field, unit in zip(panels, measures, ["head_deg","pitch_speed"], ["deg","dps"]):
            trace = [s[field] for s in path]
            panel.plot(ts+shift, trace, color=color, label=key)
            for clock, delay in [("host",0), ("shifted",shift)]:
                residual = np.interp(t-delay, ts, trace) - measured
                row[f"{key}_{clock}_{unit}"] = np.sqrt(np.mean(residual**2))
    panels[0].set_ylabel(f"{group}\n{row['initial_deg']:.0f}° to {target:g}°")
    scores.append(row)
axes[0,0].set_title("Head encoder (°)")
axes[0,1].set_title("Pitch gyro (°/s)")
for ax in axes[-1]:
    ax.set(xlim=(0,1), xlabel="Seconds after host command")
fig.suptitle(f"Unit 3 held-out lowers · common {shift*1000:.0f} ms offset for both replays")
fig.legend(*axes[0,0].get_legend_handles_labels(), loc="outside lower center", ncol=3)
fig.savefig(here/"lowering.png", dpi=140)
with (here/"lowering.csv").open("w") as stream:
    writer = csv.DictWriter(stream, fieldnames=scores[0], lineterminator="\n")
    writer.writeheader()
    writer.writerows(scores)
for group in ("Off dock","Partial"):
    subset = [r for r in scores if r["group"]==group]
    for metric in ("host_deg","host_dps","shifted_deg","shifted_dps","angle_matched_dps"):
        pairs = np.array([[r[k+"_"+metric] for k in ("frozen","faster")] for r in subset])
        print(group, metric, np.median(pairs,axis=0), sum(pairs[:,1]<pairs[:,0]), len(subset))

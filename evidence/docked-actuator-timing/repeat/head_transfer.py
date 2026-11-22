"""Test a receipt-clock shift on held-out head strokes with fixed actuator dynamics."""
import csv
import json
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from analyze import GyroReplay, params, recording, replay

def moves(path):
    with path.open() as stream:
        return [r for r in csv.DictReader(stream) if r["joint"] == "head"]

def paired(rows):
    for r in rows:
        if r["source"] != "Frozen":
            sim = next(s for s in rows if s["source"] == "Frozen" and
                       (s["bot"], s["command_s"]) == (r["bot"], r["command_s"]))
            yield r, sim

def main():
    docked = moves(HERE / "moves.csv")
    # One common shift, both directions. Freeze before reading any unit 3 scores.
    shift = np.median([float(r["onset_ms"]) - float(s["onset_ms"])
                       for r, s in paired(docked) if r["bot"] in ("unit 1", "unit 2")]) / 1000
    fig, axes = plt.subplots(2, 2, figsize=(10, 7), sharex=True, sharey=True, layout="constrained")
    scores = []
    for col, (setting, rows, path) in enumerate([
        ("Docked", docked, HERE / "unit3.jsonl"),
        ("Off dock", moves(HERE.parent / "moves.csv"), HERE.parent / "unit3.jsonl")
    ]):
        states = [r for r in map(json.loads, path.read_text().splitlines()) if r["k"] == "s"]
        for row, old in paired(rows):
            if row["bot"] != "unit 3":
                continue
            command = float(row["command_s"])
            upward = row["direction"] == "up"
            near = [s for s in states if -.3 <= s["t"] - command <= 1.53]
            t = np.array([s["t"] - command for s in near])
            angle = np.degrees([s["head"] for s in near])
            initial = replay._state_at(near, command)
            sim = GyroReplay(values=dict(params.values(params.load(model="vector_2")), **replay.OPEN_TABLE),
                             head_deg=np.degrees(initial["head"]), lift_mm=initial["lift"], noise_seed=0)
            segment = recording.Segment("head", "head", 0, 1.55, target=45 if upward else -22)
            trajectory = replay._run(sim, segment, .32)
            ts = (1 + np.arange(len(trajectory))) * sim.dt - .32
            predicted = [sample["head_deg"] for sample in trajectory]
            mask = (t >= 0) & (t <= 1)
            baseline = np.interp(t, ts, predicted)
            delayed = np.interp(t - shift, ts, predicted)
            def rmse(pred):
                return np.sqrt(np.mean((pred[mask] - angle[mask]) ** 2))
            gyro = -np.degrees([s["gyro"][1] for s in near])
            gyro -= np.median(gyro[t < 0])
            model_gyro = np.array([s["pitch_speed"] for s in trajectory])
            model_gyro -= np.median(model_gyro[ts < 0])
            def gyro_rmse(delay):
                error = np.interp(t[mask] - delay, ts, model_gyro) - gyro[mask]
                return np.sqrt(np.mean(error ** 2))
            scores.append(dict(setting=setting, direction=row["direction"], command_s=command,
                               shift_ms=1000*shift, baseline_deg=rmse(baseline), shifted_deg=rmse(delayed),
                               baseline_dps=gyro_rmse(0), shifted_dps=gyro_rmse(shift)))
            ax = axes[int(not upward), col]
            for a, color, label, style in [(angle, "#222222", "Measured", "o-"),
                                           (baseline, "#bf640e", "Frozen", "--"),
                                           (delayed, "#2068a2", "Shifted", "-")]:
                ax.plot(t, a, style, color=color, alpha=.65, ms=2, label=label)
            ax.set(title=f"{setting} · head {row['direction']}", xlim=(-.1, 1), ylabel="Head angle (°)")
    for ax in axes[-1]:
        ax.set_xlabel("Seconds from host command")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles[:3], labels[:3], loc="outside lower center", ncol=3)
    fig.suptitle(f"Unit 3 held out · {shift*1000:.0f} ms receipt shift learned on units 1 and 2")
    fig.savefig(HERE / "head-transfer.png", dpi=160)
    with (HERE / "head-transfer.csv").open("w") as f:
        w = csv.DictWriter(f, fieldnames=scores[0], lineterminator="\n")
        w.writeheader()
        w.writerows(scores)
    for s in scores:
        print(s)

if __name__ == "__main__":
    main()

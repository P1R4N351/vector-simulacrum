"""Compare real endpoint measurements across independent unit 3 recordings."""
import csv
import json
from pathlib import Path
from statistics import median

import matplotlib.pyplot as plt

folder = Path(__file__).resolve().parent
evidence = folder.parent
rows = []
for group, file in (("Full stroke", "unit3.jsonl"), ("Full stroke repeat", "repeat/unit3.jsonl"),
                    ("Slow full stroke", "repeat/slow3.jsonl")):
    records = [json.loads(line) for line in (evidence / "docked-actuator-timing" / file).read_text().splitlines()]
    commands = [r for r in records if r["k"] == "c" and r.get("name") in ("head", "lift")]
    for cmd, nxt in zip(commands, commands[1:]):
        if cmd.get("norm") != 1:
            continue
        # Use the final quiet half-second before the following command.
        samples = [r["lift"] for r in records if r["k"] == "s" and nxt["t"]-.5 <= r["t"] < nxt["t"]]
        if samples:
            rows.append((group, cmd["t"], median(samples), len(samples)))
with (folder / "holds.csv").open() as stream:
    for r in csv.DictReader(stream):
        if float(r["command_fraction"]) == 1:
            rows.append(("Quarter stroke", r["hold"], float(r["lift_mm"]), int(r["samples"])))
with (evidence / "unit3-native-pickup/rotation.csv").open() as stream:
    for r in csv.DictReader(stream):
        if r["action"] == "turn outward":
            rows.append(("Carry / " + r["condition"], r["trial"], float(r["start_lift_mm"]), ""))
with (folder / "full-raise-comparison.csv").open("w") as stream:
    writer = csv.writer(stream)
    writer.writerow(["condition", "command_seconds_or_trial", "encoder_mm", "hold_samples"])
    writer.writerows(rows)
fig, ax = plt.subplots(figsize=(8,4), constrained_layout=True)
for i, group in enumerate(dict.fromkeys(r[0] for r in rows)):
    values = [r[2] for r in rows if r[0] == group]
    ax.scatter(values, [i + .09*(j-(len(values)-1)/2) for j in range(len(values))], color="steelblue")
    ax.set_yticks(range(i+1), list(dict.fromkeys(r[0] for r in rows))[:i+1])
ax.axvline(92, color="gray", linestyle=":", label="SDK full-height target")
ax.set_xlabel("SDK lift encoder height (mm)")
ax.legend()
fig.savefig(folder / "full-raise-comparison.png", dpi=160)

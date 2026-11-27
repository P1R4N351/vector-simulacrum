"""Replay the prospective workshop validation with the frozen lift model."""
import csv
import json
from statistics import median
import matplotlib.pyplot as plt
from lift_ack import here, measure

events = [json.loads(line) for line in (here / "workshop-validation.jsonl").read_text().splitlines()]
rows = measure(events, 2, False)
with (here / "workshop-validation.csv").open("w") as output:
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(["target_mm", "sdk_s", "physical_settled_s", "earlier_s", "revised_s"])
    writer.writerows([[r[i] for i in (3, 6, 7, 8, 10)] for r in rows])
fig, ax = plt.subplots(figsize=(8, 4), layout="constrained")
for i, name, marker in [(6, "SDK response", "o"), (7, "Physical in-position flag", "+"),
                        (8, "Earlier simulator", "v"), (10, "Revised simulator", "s")]:
    ax.scatter(range(1, len(rows)+1), [r[i] for r in rows], label=name, marker=marker, s=58)
ax.set(xticks=range(1, len(rows)+1), xticklabels=["Raise", "Lower"] * 3,
       ylabel="Seconds after lift command", title="Workshop Vector · three docked lift cycles")
ax.legend(ncols=2)
fig.savefig(here / "workshop-validation.png", dpi=160)
states = [r for r in events if r['k']=='s']
print(len(states),all(r['dock'] for r in states),[round(1000*median(abs(r[k]-r[6]) for r in rows)) for k in (8,10)])

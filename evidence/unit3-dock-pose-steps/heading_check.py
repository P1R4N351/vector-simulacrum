"""Compare the canonical outward-heading rule with physical SDK reports."""
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
def error_deg(angle):
    return math.degrees(math.atan2(math.sin(angle), math.cos(angle)))

with (root/"heading-inputs.csv").open() as stream:
    rows = sorted(csv.DictReader(stream), key=lambda r: int(r["trial"][1:]))
fig, ax = plt.subplots(figsize=(8.8, 4.1), layout="constrained")
for i, row in enumerate(rows):
    w, x, y, z = [float(row[f"charger_q{k}"]) for k in range(4)]
    # Outward direction is the negative first column of the charger rotation.
    target = math.atan2(-2*(x*y+w*z), -(1-2*(y*y+z*z)))
    before, after = [error_deg(float(row[key])-target) for key in ("last_off_rad","first_on_rad")]
    ax.plot([i,i],[before,after],color="0.75",zorder=1)
    ax.scatter(i,before,color="tab:blue",label="Last off" if not i else None)
    ax.scatter(i,after,color="tab:orange",marker="D",label="First on" if not i else None)
    print(row["trial"], f"{before:.3f}", f"{after:.3f}")
ax.axhline(0,linewidth=.7,color="black")
ax.set_xticks(range(len(rows)),[r["trial"] for r in rows])
ax.set(ylabel="SDK heading minus predicted heading (degrees)",title="Heading correction at charger contact • W35 held out")
ax.legend()
fig.savefig(root/"heading-transfer.png",dpi=160)

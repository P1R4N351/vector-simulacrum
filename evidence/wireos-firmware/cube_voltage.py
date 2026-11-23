"""Compare the fixed shelf ADC word with direct BLE rail notifications."""
import csv
from pathlib import Path
import statistics

import matplotlib.pyplot as plt

here = Path(__file__).resolve().parent
direct = here.parent/"unit3-wheelstand/twin-timing"
fig, axes = plt.subplots(1,4,figsize=(11,3.4),layout="constrained",sharey=True)
for ax,name in zip(axes,("led-accel","led-accel-repeat","dark-direct","blink-clock-receipts")):
    with (direct/(name+".csv")).open() as f:
        packets = [(float(r["sec"]),bytes.fromhex(r["value"])) for r in csv.DictReader(f)
                   if r["kind"]=="packet" and r["value"].startswith("04")]
    assert all(len(p)==4 for _,p in packets)
    counts = [int.from_bytes(p[2:],"little") for _,p in packets]
    ax.scatter([t-packets[0][0] for t,_ in packets],counts,label="Physical cube",color="#20282b")
    ax.axhline(426,color="#a35c20",ls="--",label="Generic cube")
    ax.axhline(444,color="#197577",label="Shelf cube",zorder=0)
    ax.set_title(name.replace("-"," "))
    print(name,len(counts),min(counts),max(counts),statistics.mean(abs(c-444) for c in counts))
axes[0].set_ylabel("Battery ADC counts")
fig.supxlabel("Seconds since first voltage notification in each session")
fig.legend(*axes[0].get_legend_handles_labels(),loc="outside upper center",ncol=3)
fig.savefig(here/"cube-voltage.png",dpi=150)

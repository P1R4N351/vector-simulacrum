"""Fit direction-specific 75% settled residuals on blocks 1–2 only."""
import csv
from pathlib import Path
from statistics import mean
import matplotlib.pyplot as plt

folder = Path(__file__).resolve().parent
def read(name):
    with name.open() as stream:
        return list(csv.DictReader(stream))
middle = [r for r in read(folder/"replay.csv") if r["phase"] == "middle"]
correction = {prep: -mean(float(r["sim_minus_real_mm"]) for r in middle
                          if int(r["block"]) < 2 and float(r["prep_fraction"]) == prep) for prep in [.5, 1.]}
trials = [(f"Block {int(r['block'])+1}", float(r["prep_fraction"]), float(r["sim_hold_mm"]),
           float(r["measured_hold_mm"])) for r in middle if int(r["block"]) >= 2]
old = folder.parent/"unit3-docked-fork-sweep"
for hold, error in zip(read(old/"holds.csv")[1:], read(old/"replay-errors.csv")):
    if float(hold["command_fraction"]) == .75:
        measured = float(hold["lift_mm"])
        prep = .5 if int(hold["hold"]) in [3,11] else 1.
        trials.append((f"Sweep {error['cycle']}",prep,measured+float(error["hold_error_sim_minus_real_mm"]),measured))
fig, ax = plt.subplots(figsize=(9,4), constrained_layout=True)
with (folder/"transfer.csv").open("w") as stream:
    writer = csv.writer(stream)
    writer.writerow(["set","prep","frozen_error_mm","corrected_error_mm"])
    for i,(label,prep,pred,obs) in enumerate(trials):
        writer.writerow([label,prep,pred-obs,pred+correction[prep]-obs])
        ax.scatter(i-.08,pred-obs,color="slategray",label="Frozen" if i==0 else None)
        ax.scatter(i+.08,pred+correction[prep]-obs,color="darkcyan",label="Direction correction" if i==0 else None)
ax.axhline(0,color="gray",lw=.8)
ax.set_xticks(range(len(trials)),[f"{r[0]}\nfrom {r[1]:.0%}" for r in trials])
ax.set_ylabel("75% settled encoder error (mm)")
ax.legend()
fig.savefig(folder/"transfer.png",dpi=160)

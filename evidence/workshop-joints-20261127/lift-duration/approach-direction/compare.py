"""Test transfer of endpoint corrections from 44 mm to 52 mm."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
def read(name):
    with (here / name).open() as f:
        return list(csv.DictReader(f))

events = read("recording.csv")
states = [r for r in events if r["event"] == "state"]
model = read("replay.csv")
t = np.array([float(r["s"]) for r in states])
assert np.array_equal(t,[float(r["s"]) for r in model])
assert all(int(r["on_dock"]) for r in states+model)
commands = [r for r in events if r["event"]=="command"]
assert len(commands)==25
assert [r["code"] for r in events if r["event"]=="return"]==["0"]*25

fig,axes = plt.subplots(2,2,sharex=True,sharey="row",layout="constrained",figsize=(9,6))
records = []
for j in range(1,24,2):
    target = int(commands[j]["target_mm"])
    up = int(commands[j-1]["target_mm"]) < target
    x = t-float(commands[j]["s"])
    assert float(commands[j+1]["s"])-float(commands[j]["s"])>1.1
    scores = []
    for col,(rows,style,label) in enumerate([(states,"-","Physical"),(model,"--","Canonical")]):
        y = np.array([float(r["lift_mm"]) for r in rows])
        settled = y[(x>=.9)&(x<=1.1)]
        assert len(settled)>=2
        scores.append(np.median(settled)-target)
        ax=axes[int(target==52),int(not up)]
        sel=(x>=-.1)&(x<=1.1)
        ax.step(x[sel],y[sel],where="post",ls=style,color=f"C{col}",alpha=.7,
                label=label if len(ax.lines)<2 else None)
        ax.set_title(f"{target} mm from {'below' if up else 'above'}")
    records.append([j//2+1,target,"up" if up else "down",*scores,scores[0]-scores[1]])
for ax in axes.flat:
    ax.legend(fontsize=8)
    ax.grid(alpha=.2)
for ax in axes[-1]:
    ax.set_xlabel("Seconds after command")
for ax in axes[:,0]:
    ax.set_ylabel("Lift height (mm)")
fig.savefig(here/"endpoints.png",dpi=160)
for target in (44,52):
    for direction in ("up","down"):
        group = np.array([r[3:] for r in records if r[1:3]==[target,direction]])
        print(target,direction,"physical,sim,error medians",np.round(np.median(group,axis=0),2))
fit = [r for r in records if r[1]==44]
for r in records:
    direction_bias = np.median([f[5] for f in fit if f[2]==r[2]]) if r[1]==52 else np.nan
    common_bias = np.median([f[5] for f in fit]) if r[1]==52 else np.nan
    r.extend((r[5]-common_bias,r[5]-direction_bias))
with (here/"endpoints.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("trial target_mm direction physical_minus_target_mm sim_minus_target_mm physical_minus_sim_mm common_residual_mm direction_residual_mm".split())
    w.writerows(records)
held=np.array([r[5:] for r in records if r[1]==52])
print("held MAE frozen/common/direction",np.round(np.mean(np.abs(held),axis=0),2),"N SDK",len(t))
prior=read("../elevated-target/scores.csv")
bias=np.median([r[5] for r in fit])
for target in (32,40):
    pairs={}
    for r in prior:
        if int(r["target_mm"])==target:
            pairs.setdefault(r["trial"],{})[r["trace"]]=float(r["settled_minus_target_mm"])
    residual=np.array([v["Physical"]-v["Canonical"] for v in pairs.values()])
    print("prior target",target,"frozen/common MAE",np.round([np.mean(abs(residual)),np.mean(abs(residual-bias))],2))

"""Compare physical SDK returns with action completion in the browser simulation."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
def read(path):
    with path.open() as file:
        return list(csv.DictReader(file))

scores = []
heights = []
fig, axes = plt.subplots(1,2,figsize=(10,4),layout="constrained",sharey=True)
for ax,folder in zip(axes,("approach-direction","short-repeat")):
    base = here.parent / folder
    raw = read(base/"recording.csv")
    cmd = [r for r in raw if r["event"]=="command"]
    ack = [r for r in raw if r["event"]=="return"]
    sim = read(base/"returns.csv")
    assert len(cmd)==len(ack)==len(sim)
    for i,(c,a,m) in enumerate(zip(cmd,ack,sim)):
        assert c["target_mm"]==a["target_mm"]==m["target_mm"]
        assert c["s"]==m["command_s"] and a["code"]=="0"
        assert m["result"]=="completed"
        if float(c["duration_s"])==0:
            continue
        observed = 1000*(float(a["s"])-float(c["s"]))
        predicted = 1000*float(m["sim_elapsed_s"])
        def height(origin,left,right):
            window = [float(r["lift_mm"]) for r in raw if r["event"]=="state" and
                      left <= float(r["s"])-float(origin["s"]) <= right]
            assert len(window)>=2
            return np.median(window)
        heights.append((folder,i//2+1,height(c,.9,1.1),height(a,.15,.45)))
        direction = "up" if float(cmd[i-1]["target_mm"])<float(c["target_mm"]) else "down"
        scores.append([folder,i//2+1,c["target_mm"],direction,c["duration_s"],observed,predicted,observed-predicted])
    trial = [row for row in scores if row[0]==folder]
    for col,label in [(5,"Physical SDK"),(6,"Canonical")]:
        ax.plot([r[1] for r in trial],[r[col] for r in trial],".-",label=label)
    ax.set(xlabel="Trial",title=folder.replace("-"," ").capitalize(),xticks=[1,3,5,7,9,12])
    ax.legend()
    ax.grid(alpha=.2)
axes[0].set_ylabel("Time from command to return (ms)")
fig.savefig(here/"returns.png",dpi=160)

fit = [r[7] for r in scores if r[0]=="approach-direction" and r[2]=="44"]
shift = np.median(fit)
for row in scores:
    row.append(row[7]-shift)
with (here/"scores.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("session trial target_mm direction duration_s sdk_ms canonical_ms residual_ms shifted_residual_ms".split())
    w.writerows(scores)
print("training median offset",shift)
for target,duration in [("44","0.6"),("52","0.6"),("32","0.6"),("32","0.132")]:
    s=np.array([row[5:] for row in scores if row[2]==target and row[4]==duration])
    print(target,duration,"medians physical/sim/signed",np.round(np.median(s,axis=0)[:3]),
          "unshifted/shifted MAE",np.round(np.mean(abs(s[:,2:]),axis=0)))

with (here/"heights.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["session","trial","height_at_0p9_to_1p1_s_mm","height_150_to_450_ms_after_return_mm"])
    w.writerows(heights)

"""Compare two source-based lift derivative diagnostics with frozen complete-session replays."""
import csv
from pathlib import Path
from statistics import mean
import sys

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent/"friction"))
from score import evaluate

rows=[]
for source,split in [("approach-direction","train"),("status-52","test")]:
    for mode in ("baseline","target","error"):
        variant=here/f"{source}-{mode}"
        if mode!="baseline" and not (variant/"replay.csv").exists():
            print(source,mode,"incomplete")
            continue
        part=evaluate(source,mode,split,output=here.parent/source if mode=="baseline" else variant)
        for r in part:
            r[1]=mode
        rows.extend(part)
        print(source,mode,"raise/lower height RMSE",
              [round(mean(r[4] for r in part if r[3]==d),3) for d in ("raise","lower")],
              "flag MAE",round(mean(abs(r[5]) for r in part)) if split=="test" else "n/a")
with (here/"scores.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("split mode trial direction height_rmse_mm flag_error_ms".split())
    w.writerows(rows)

import matplotlib.pyplot as plt
from score import read
source=here.parent/"status-52"
events=read(source/"recording.csv")
real=[r for r in events if r["event"]=="state"]
cmd=[r for r in events if r["event"]=="command"]
fig,axes=plt.subplots(2,4,layout="constrained",figsize=(12,5),sharex=True,sharey=True)
for j,ax in enumerate(axes.flat):
    cue=float(cmd[2*j+1]["s"])
    direction="raise" if float(cmd[2*j]["target_mm"])<52 else "lower"
    for label,series,style in [("Physical",real,"k-"),("Canonical",read(source/"replay.csv"),"C0--")]+[
        (mode,read(here/f"status-52-{mode}"/"replay.csv"),style) for mode,style in (("target","C1:"),("error","C2-."))
        if (here/f"status-52-{mode}"/"replay.csv").exists()]:
        sel=[(float(r["s"])-cue,float(r["lift_mm"])) for r in series if -.08<=float(r["s"])-cue<=1.4]
        ax.plot(*zip(*sel),style,lw=1.4,label=label)
    ax.set_title(f"{j+1} · {direction}")
    ax.grid(alpha=.2)
axes.flat[0].legend(fontsize=7)
fig.supxlabel("Seconds after command")
fig.supylabel("Lift height (mm)")
fig.savefig(here/"held-out.png",dpi=160)

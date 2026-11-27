"""Compare full-scale torque diagnostics against two independent docked sessions."""
import csv
from pathlib import Path
from statistics import mean
import sys

import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent/"friction"))
from score import evaluate, read

results=[]
fig,axes=plt.subplots(2,4,figsize=(12,5.2),sharex=True,sharey=True,layout="constrained")
for source,split in [("approach-direction","train"),("status-52","test")]:
    src=here.parent/source
    events=read(src/"recording.csv")
    cmds=[r for r in events if r["event"]=="command"]
    for mode in ("baseline","proportional","both"):
        folder=src if mode=="baseline" else here/f"{source}-{mode}"
        part=evaluate(source,mode,split,output=folder)
        results.extend(part)
        print(source,mode,*(round(mean(r[4] for r in part if r[3]==d),3) for d in ("raise","lower")),
              round(mean(abs(r[5]) for r in part),1) if split=="test" else "")
        if split!="test":
            continue
        for i,ax in enumerate(axes.flat):
            cue=float(cmds[2*i+1]["s"])
            traces=[(mode,{"baseline":"C0--","proportional":"C1:","both":"C2-."}[mode],read(folder/"replay.csv"))]
            if mode=="baseline":
                traces.insert(0,("Physical","k-",[r for r in events if r["event"]=="state"]))
            for label,style,rows in traces:
                series=[(float(r["s"])-cue,float(r["lift_mm"])) for r in rows if -.08<=float(r["s"])-cue<=1.4]
                ax.plot(*zip(*series),style,lw=1.3,label=label)
            ax.set_title(f"{i+1} · {part[i][3]}")
axes.flat[0].legend(fontsize=7)
fig.supxlabel("Seconds after lift command")
fig.supylabel("Encoder height (mm)")
fig.savefig(here/"held-out.png",dpi=160)
with (here/"scores.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("session mode trial direction height_rmse_mm flag_error_ms".split())
    w.writerows(results)

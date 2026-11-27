"""Compare identical lowers reached by continuation and reversal on the dock."""
import csv
from pathlib import Path
from statistics import median

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
def read(name):
    with (here/name).open() as f:
        return list(csv.DictReader(f))

events=read("recording.csv")
physical=[r for r in events if r["event"]=="state"]
model=read("replay.csv")
commands=[r for r in events if r["event"]=="command"]
acks=[r for r in events if r["event"]=="return"]
model_acks=read("returns.csv")
assert len(commands)==len(acks)==len(model_acks)==37
assert all(r["code"]=="0" for r in acks) and all(r["result"]=="completed" for r in model_acks)
assert all(int(r["on_dock"]) for r in physical)
t=np.array([float(r["s"]) for r in physical])
assert np.array_equal(t,[float(r["s"]) for r in model])
fig,axes=plt.subplots(2,2,figsize=(10,6),layout="constrained",sharex=True,sharey=True)
records=[]
for j in range(2,36,3):
    trial=j//3+1
    assert int(commands[j-1]["target_mm"])==64 and int(commands[j]["target_mm"])==52
    history="reverse" if int(commands[j-2]["target_mm"])==52 else "continue"
    cue=float(commands[j]["s"])
    elapsed=t-cue
    sel=(elapsed>=-.18)&(t<float(commands[j+1]["s"]))
    x=elapsed[sel]
    for col,(label,rows) in enumerate([("Physical",physical),("Canonical",model)]):
        y=np.array([float(r["lift_mm"]) for r in rows])[sel]
        flags=np.array([int(r["in_position"]) for r in rows])[sel]
        clear=np.flatnonzero((x>=0)&(flags==0))[0]
        on=clear+np.flatnonzero(flags[clear:])[0]
        ret=float(acks[j]["s"])-cue if col==0 else float(model_acks[j]["sim_elapsed_s"])
        start=median(y[x<0])
        records.append([trial,history,label,start,1000*x[on-1],1000*x[on],1000*ret,int(all(int(r["on_dock"]) for r,active in zip(rows,sel) if active))])
        ax=axes[int(history=="continue"),col]
        ax.plot(x,y,lw=1,alpha=.8)
        ax.scatter(x[on],y[on],s=20)
        ax.set_title(f"{label} · {history} lowering")
        ax.set_xlim(-.15,1.65)
        ax.grid(alpha=.15)
for ax in axes[-1]:
    ax.set_xlabel("Seconds after 52 mm command")
for ax in axes[:,0]:
    ax.set_ylabel("Lift height (mm)")
fig.savefig(here/"history.png",dpi=160)
with (here/"scores.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("trial history trace start_mm last_clear_ms first_set_ms return_ms continuous_dock".split())
    w.writerows(records)
for label in ("Physical","Canonical"):
    contrasts=[]
    for n in range(6):
        pair=[r for r in records if r[2]==label and r[0] in (2*n+1,2*n+2)]
        vals={r[1]:r[5] for r in pair}
        contrasts.append(vals["reverse"]-vals["continue"])
    for history in ("reverse","continue"):
        a=np.array([r[3:] for r in records if r[1:3]==[history,label]])
        print(label,history,"median start/clear/set/return",np.round(np.median(a,axis=0),1),"individual sets",np.round(a[:,2]),"start heights",np.round(a[:,0],2))
    print("paired differences ms",np.round(contrasts),"median",median(contrasts))
print(len(t),"physical packets")
# Predict a held-out pair from the other five adjacent pairs.
real=[r for r in records if r[2]=="Physical"]
predictions=[]
for i,r in enumerate(real):
    train=[s for j,s in enumerate(real) if i//2!=j//2]
    x=np.array([s[3] for s in train])
    y=np.array([s[5] for s in train])
    slope,intercept=np.polyfit(x,y,1)
    predictions.append([r[0],r[5],median(y),median(s[5] for s in train if s[1]==r[1]),intercept+slope*r[3]])
p=np.array(predictions)
print("leave-pair-out mean absolute flag errors; common, history, start height",np.round(np.mean(np.abs(p[:,2:]-p[:,1:2]),axis=0)))
with (here/"prediction.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("trial actual_ms common_median_ms history_median_ms start_height_linear_ms".split())
    w.writerows(predictions)
# Apply the session models to the four lowers in the separately recorded flag session.
other=here.parent/"status-52"
with (other/"recording.csv").open() as f:
    old=list(csv.DictReader(f))
oldcmd=[r for r in old if r["event"]=="command"]
with (other/"scores.csv").open() as f:
    checks=[r for r in csv.DictReader(f) if r["trace"]=="Physical" and r["direction"]=="down"]
slope,intercept=np.polyfit([r[3] for r in real],[r[5] for r in real],1)
validation=[]
for r in checks:
    cue=float(oldcmd[2*int(r["trial"])-1]["s"])
    height=median(float(s["lift_mm"]) for s in old if s["event"]=="state" and -.18<=float(s["s"])-cue<0)
    validation.append([int(r["trial"]),height,float(r["first_set_ms"]),median(v[5] for v in real),
                       median(v[5] for v in real if v[1]=="reverse"),intercept+slope*height])
v=np.array(validation)
print("separate-session MAEs",np.round(np.mean(abs(v[:,3:]-v[:,2:3]),axis=0)))
with (here/"separate-session.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("trial start_mm actual_ms common_median_ms reverse_median_ms height_linear_ms".split())
    w.writerows(validation)

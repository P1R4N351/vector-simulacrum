"""Select lift Coulomb friction on an earlier recording and score independent strokes."""
import csv
import math
from pathlib import Path
from statistics import mean, median
import sys

here = Path(__file__).resolve().parent
torques = ("0", "0.003", "0.009", "0.018")
def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))

def evaluate(session, torque, prefix):
    src=here.parent/session
    out=src if torque=="0" else here/f"{prefix}-{torque}"
    events=read(src/"recording.csv")
    real=[r for r in events if r["event"]=="state"]
    replay=read(out/"replay.csv")
    returns=read(out/"returns.csv")
    commands=[r for r in events if r["event"]=="command"]
    assert len(commands)==len(returns) and all(r["result"]=="completed" for r in returns)
    assert [r["s"] for r in real]==[r["s"] for r in replay]
    assert all(int(r["on_dock"]) for r in real+replay)
    scores=[]
    for i,c in enumerate(commands):
        if float(c["duration_s"])==0:
            continue
        cue=float(c["s"])
        end=cue+1.4
        assert end<float(commands[i+1]["s"])
        pairs=[(a,b) for a,b in zip(real,replay) if cue<=float(a["s"])<=end]
        rmse=math.sqrt(mean((float(a["lift_mm"])-float(b["lift_mm"]))**2 for a,b in pairs))
        direction="raise" if float(commands[i-1]["target_mm"])<float(c["target_mm"]) else "lower"
        flag_error=None
        if prefix=="test":
            def first_set(rows):
                clear=False
                for row in rows:
                    t=float(row["s"])
                    if cue<=t<float(commands[i+1]["s"]):
                        if row["in_position"]=="0":
                            clear=True
                        elif clear:
                            return t
                raise ValueError("No in-position transition")
            flag_error=1000*(first_set(replay)-first_set(real))
        scores.append([prefix,torque,i//2+1,direction,rmse,flag_error])
    return scores

if __name__=="__main__":
    train=[]
    for torque in torques:
        train.extend(evaluate("approach-direction",torque,"train"))
    def loss(torque):
        return mean(r[4] for r in train if r[1]==torque)
    selected=min(torques[1:],key=loss)
    print("training losses",[(k,round(loss(k),3)) for k in torques])
    print("best nonzero",selected)
    if "--select" in sys.argv:
        sys.exit(0)
    test=[]
    for torque in ("0",selected):
        rows=evaluate("status-52",torque,"test")
        test.extend(rows)
        print("holdout",torque,"height RMSE",mean(r[4] for r in rows),"flag MAE",mean(abs(r[5]) for r in rows))
    with (here/"scores.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("split friction_Nm trial direction height_rmse_mm flag_error_ms".split())
        w.writerows(train+test)
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(10,3.7),layout="constrained")
    axes[0].plot([1000*float(k) for k in torques],[loss(k) for k in torques],".-")
    axes[0].set(xlabel="Lift friction (mN m)",ylabel="Mean stroke RMSE (mm)",title="Earlier session · 12 strokes")
    for col,metric,title in [(1,4,"Lift height RMSE (mm)"),(2,5,"Absolute flag error (ms)")]:
        for torque in ("0",selected):
            rows=[r for r in test if r[1]==torque]
            axes[col].plot([r[2] for r in rows],[abs(r[metric]) for r in rows],".-",label=f"{1000*float(torque):g} mN m")
        axes[col].set(xlabel="Held-out stroke",ylabel=title)
        axes[col].legend()
    for ax in axes: ax.grid(alpha=.2)
    fig.savefig(here/"friction.png",dpi=160)

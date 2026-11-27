"""Measure target arrival, return displacement and terminal position during docked wake."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root=Path(__file__).resolve().parent


def stream(path):
    with path.open() as f:
        rows=list(csv.DictReader(f))
    events={r["event"]:float(r["receipt_s"]) for r in rows if r["event"]!="state"}
    data=np.array([[float(r[k]) for k in ("receipt_s","head_deg","gyro_y","status")] for r in rows if r["event"]=="state"])
    return {k:v-events["granted"] for k,v in events.items()},data[:,0]-events["granted"],data[:,1:]


def main():
    fig,axes=plt.subplots(3,3,figsize=(13,7),sharex=True,sharey="row",layout="constrained")
    scores=[]
    for col in range(3):
        events,t,raw=stream(root/f"wake-raise-{col+1}.csv")
        head,gyro,flags=raw.T
        flags=flags.astype(int)
        cue=events["raise"]
        ack=events["raise_return_0"]
        end=events["restore"]
        initial=head[t<0][-1]
        hold=(t>=ack)&(t<end)
        arrival=np.flatnonzero((t>=cue)&(t<end)&(abs(head-22)<=2))
        before=t<ack
        awake=(flags&64)!=0
        refdir=root.parent/"blind-grants"
        references=[]
        for i in (1,2):
            _,ref_t,ref=stream(refdir/f"calm-cross-{i}.csv")
            offset=initial-ref[ref_t<0,0][-1]
            references.append(np.interp(t,ref_t,ref[:,0]+offset))
            for j in (0,1):
                axes[j,col].plot(ref_t,ref[:,j]+(offset if j==0 else 0),color=".72",lw=.9,
                                 label="Uncommanded reference" if i==1 else None)
        for ax,signal in zip(axes[:,col],(head,gyro,awake)):
            ax.plot(t,signal,color="C0",lw=1.2)
            ax.axvline(cue,ls="--",color="C1")
            ax.axvline(ack,ls=":",color="C1")
            ax.set_xlim(-.2,8.99)
        axes[0,col].axhline(22,color="black",ls=":",lw=.7)
        axes[0,col].set_title(f"Run {col+1} · raise at {cue:.1f} s")
        axes[2,col].set_xlabel("Seconds after control grant")
        follow=(t>=ack+.5)&(t<min(end,8))
        ref_error=np.sqrt(np.mean((np.mean(references,axis=0)[follow]-head[follow])**2))
        target_error=np.sqrt(np.mean((22-head[follow])**2))
        scores.append([col+1,cue,ack-cue,t[arrival[0]]-cue if arrival.size else None,
                       head[before][-1],np.min(head[hold]),np.max(head[hold]),head[hold][-1],
                       np.max(np.abs(gyro[hold])),sum(awake[hold]),sum(hold),
                       int(bool(flags[t<0][-1]&1024)),ref_error,target_error])
    for ax,label in zip(axes[:,0],("Head (degrees)","Gyro Y (rad/s)","Animating")):ax.set_ylabel(label)
    axes[0,0].legend(fontsize=8)
    fig.savefig(root/"traces.png",dpi=160)
    with (root/"scores.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("run cue_s ack_latency_s first_within_2deg_s angle_last_before_ack hold_min_deg hold_max_deg hold_last_deg hold_max_abs_gyro hold_animating hold_reports calm_before_grant reference_resume_rmse_deg target_hold_rmse_deg".split())
        w.writerows(scores)


if __name__=="__main__": main()

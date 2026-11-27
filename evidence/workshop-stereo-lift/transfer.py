"""Evaluate the previously estimated SDK completion offset on distinct sessions."""
import csv
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np

root=Path(sys.argv[1]) if len(sys.argv)>1 else Path("../workshop-joints-20261127/lift-duration")
offset=.1252926826477051
def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))
out=[]
for folder in ["short-repeat","approach-direction","status-52","history-52"]:
    trace=read(root/folder/"recording.csv")
    replay=read(root/folder/"returns.csv")
    commands=[r for r in trace if r["event"]=="command"]
    returns=[r for r in trace if r["event"]=="return"]
    assert len(commands)==len(returns)==len(replay)
    states=[r for r in trace if r["event"]=="state"]
    for c,r,s in zip(commands,returns,replay):
        start=float(c["s"])
        assert start==float(s["command_s"]) and float(r["s"])>=start
        assert c["target_mm"]==r["target_mm"]==s["target_mm"] and r["code"]=="0" and s["result"]=="completed"
        initial=float(next(row for row in reversed(states) if float(row["s"])<=start)["lift_mm"])
        target=float(c["target_mm"])
        physical=float(r["s"])-start
        sim=float(s["sim_elapsed_s"])
        direction="raise" if target>initial+2 else "lower" if target<initial-2 else "near target"
        out.append([folder,start,float(c["duration_s"]),direction,initial,target,physical,sim,1000*(sim-physical),1000*(sim+offset-physical)])
with open("transfer-scores.csv","w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["session","command_s","duration_s","direction","start_mm","target_mm","physical_s","sim_s","base_error_ms","corrected_error_ms"])
    w.writerows(out)
fig,axes=plt.subplots(1,3,figsize=(11,4),layout="constrained",sharey=True)
for ax,duration in zip(axes,[0,.132,.6]):
    group=[r for r in out if r[2]==duration]
    for direction,color in [("raise","tab:blue"),("lower","tab:orange"),("near target","gray")]:
        subset=[r for r in group if r[3]==direction]
        if not subset:continue
        v=np.array([r[-2:] for r in subset]);print(duration,direction,len(v),np.mean(abs(v),axis=0),np.median(v,axis=0))
        ax.scatter([0]*len(v),v[:,0],color=color,s=12,alpha=.6)
        ax.scatter([1]*len(v),v[:,1],color=color,s=12,alpha=.6,label=direction)
        for pair in v:ax.plot([0,1],pair,color=color,alpha=.12)
    ax.axhline(0,color="black",lw=.7)
    ax.set(title="Default speed" if duration==0 else f"{duration:g} s requested",xticks=[0,1],xticklabels=["Canonical","+125 ms"])
    ax.legend(fontsize=8)
axes[0].set_ylabel("Simulated minus physical SDK return (ms)")
fig.savefig("transfer.png",dpi=165)
v=np.array([r[-2:] for r in out]);print("all",len(v),np.mean(abs(v),axis=0),np.sum(abs(v[:,1])>abs(v[:,0])))
# Diagnostic only. Replace the simulated completion instant with a measured flag.
with open("flag-control.csv","w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["session","command_s","target_mm","in_position_s","sdk_return_s","corrected_sim_error_ms","observed_flag_plus_62ms_error_ms"])
    flag_scores=[]
    for folder in ["status-52","history-52"]:
        trace=read(root/folder/"recording.csv")
        group=[r for r in out if r[0]==folder]
        for i,r in enumerate(group):
            start=r[1]
            stop=group[i+1][1] if i+1<len(group) else float("inf")
            if r[3]=="near target":continue
            states=[row for row in trace if row["event"]=="state" and start<=float(row["s"])<stop]
            deasserted=False
            for state in states:
                if state["in_position"]=="0":
                    deasserted=True
                elif deasserted and state["in_position"]=="1":
                    settled=float(state["s"])-start
                    error=1000*(settled+.062-r[6])
                    w.writerow([folder,start,r[5],settled,r[6],r[-1],error])
                    flag_scores.append([r[-1],error])
                    break
    print("observed flag control",len(flag_scores),np.mean(abs(np.array(flag_scores)),axis=0))

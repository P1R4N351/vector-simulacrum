"""Fit one common receipt-time offset on sweep one, then freeze it."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from simulacrum import params
from simulacrum.robot import VectorSim

here = Path(__file__).resolve().parent
sweep = here.parent/"unit3-docked-fork-sweep"
state = np.genfromtxt(sweep/"state.csv.gz", names=True, delimiter=",")
moves = np.genfromtxt(sweep/"holds.csv", names=True, delimiter=",")
failed = np.genfromtxt(here/"dock-status.csv", names=True, delimiter=",")

def replay(commands, end, lift, head, duration):
    sim = VectorSim(params.values(params.load(model="vector_2")), lift_mm=lift, head_deg=head)
    bits = [sim.state.lift_in_pos]
    i = 0
    while (len(bits)-1)*sim.dt <= end:
        if i<len(commands) and (len(bits)-1)*sim.dt >= commands[i][0]:
            sim.command_lift_height(commands[i][1],duration_s=duration)
            i += 1
        bits.append(sim.step().lift_in_pos)
    return np.arange(len(bits))*sim.dt,np.array(bits)

def edges(t,values,midpoint=False):
    i = np.flatnonzero(values[1:]!=values[:-1])+1
    return ((t[i]+t[i-1])/2 if midpoint else t[i]),values[i]

t=state["seconds"]
sim_t,sim_bits=replay(list(zip(moves["command_s"],32+60*moves["command_fraction"])),t[-1],32,np.degrees(state["head_rad"][0]),.7)
obs=(state["status"].astype(int)&256)!=0
st,sb=edges(sim_t,sim_bits)
pt,pb=edges(t,obs,True)
pairs=[]
for i in range(1,len(moves)):
    start=moves[i]["command_s"];end=moves[i+1]["command_s"] if i+1<len(moves) else t[-1]+.001
    for kind in (False,True):
        a=pt[(pt>=start)&(pt<end)&(pb==kind)]
        b=st[(st>=start)&(st<end)&(sb==kind)]
        if len(a)==1 and len(b)==1:
            pairs.append((i,kind,b[0],a[0],a[0]-b[0]))
lag=np.median([p[-1] for p in pairs if p[0]<=8])
print("lag",lag,"paired",len(pairs))
def score(label,t,obs,xt,y,mask):
    rates=[]
    for delay in (0,lag):
        pred=y[np.maximum(0,np.searchsorted(xt,t-delay,side="right")-1)]
        rates.append(np.mean(pred[mask]==obs[mask]))
    print(label,rates)
    return rates

groups=[]
for i in (1,2):
    start=moves[1 if i==1 else 9]["command_s"];end=moves[9]["command_s"] if i==1 else t[-1]+.001
    groups.append(score(str(i),t,obs,sim_t,sim_bits,(t>=start)&(t<end)))
ft=failed["seconds"]; fo=(failed["status"].astype(int)&256)!=0
fx,fy=replay([(0,47)],ft[-1],33.02,21.5,1)
groups.append(score("failed grip",ft,fo,fx,fy,ft>=0))
with (here/"status-lag.csv").open("w") as f:
    w=csv.writer(f);w.writerow(["move","set","sim_edge_s","physical_midpoint_s","offset_s","residual_with_delay_s"])
    for p in pairs:w.writerow([*p,p[-1]-lag])
# Entirely separate session, repeated 0-to-full and 75%-to-full raises.
import gzip,json
with gzip.open(here.parent/"unit3-paired-raise/paired.jsonl.gz","rt") as stream:
    records=[json.loads(line) for line in stream]
commands=[r for r in records if r["kind"]=="command"]
samples=[r for r in records if r["kind"]=="state"]
z=commands[0]["t"]
tt=np.array([r["t"]-z for r in samples])
obs_pair=np.array([bool(r["status"]&256) for r in samples])
tx,yy=replay([(r["t"]-z,32+60*r["pos"]) for r in commands],tt[-1],samples[0]["lift"],np.degrees(samples[0]["head"]),.7)
pair_rates=score("interleaved full raises",tt,obs_pair,tx,yy,tt>=0)
with (here/"status-lag-agreement.csv").open("w") as f:
    w=csv.writer(f); w.writerow(["recording","frozen_agreement","delayed_agreement"])
    for name,values in zip(["first sweep","second sweep","failed grip","interleaved full raises"],groups+[pair_rates]):
        w.writerow([name,*values])
fig,ax=plt.subplots(1,2,figsize=(10,4),layout="constrained")
for kind in (False,True):
    q=[p for p in pairs if p[1]==kind]
    ax[0].scatter([p[0] for p in q],[1000*p[-1] for p in q],label="Set" if kind else "Clear")
ax[0].axhline(1000*lag,ls=":",color="black");ax[0].axvline(8.5,color="gray")
ax[0].set(xlabel="Physical move",ylabel="Observed minus simulated edge (ms)");ax[0].legend()
for j,label in enumerate(("Frozen","Offset applied")):
    ax[1].bar(np.arange(4)+(.36*j-.18),np.array(groups+[pair_rates])[:,j]*100,.36,label=label)
ax[1].set(xticks=range(4),xticklabels=["Sweep 1","Sweep 2","Grip","Full raises"],ylabel="SDK sample agreement (%)",ylim=(0,100));ax[1].legend()
fig.savefig(here/"status-lag.png",dpi=160)


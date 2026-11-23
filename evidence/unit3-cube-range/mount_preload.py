"""Constrain earlier reverse-command onset with recorded pre-motion encoder reports."""
import csv
import io
import json
import subprocess

import matplotlib.pyplot as plt
import numpy as np

from recorded_mount import ROOT, read_trial, states, table


def main():
    inputs=[]
    cases=[]
    limits=table("mount-native-entry.csv")
    for contact,obs in zip(table("return-scores.csv"),table("mount-target-inputs.csv"),strict=True):
        trial,real=read_trial(contact,obs)
        name=trial["name"]
        t,out,_,_,w=real
        rows,_=states(name)
        old=[r for r in rows if float(r["receipt_s"])-contact["first_on_s"] <= t[0]]
        zero=[]
        for row in reversed(old):
            if any(float(row[k])!=0 for k in ("left_mmps","right_mmps")):
                break
            zero.append(float(row["receipt_s"])-contact["first_on_s"])
        zero=np.array(zero[::-1])
        assert zero[-1]==t[0]
        duration=limits[(limits["session"]==name)&(limits["onset"]=="last_zero")]["window_s"].item()
        end=t[0]+duration
        cases.append((trial,real,zero,end))
        for lead in [0,25,50,75,100]:
            onset=t[0]-lead/1000
            if onset < zero[0]-.000001:
                continue
            # Give the encoder the sampled idle interval before the command.
            samples=[[zero[0]-.005,0,0],[onset,-30,-30]]
            inputs.append(dict(trial,name=f"{name}_{lead}",immediate_wheels=True,wheels=samples,
                               duration_s=end-samples[0][0]+.01))
    raw=subprocess.check_output(["node",str(ROOT/"recorded_mount.mjs")],text=True,input=json.dumps(inputs))
    data=np.genfromtxt(io.StringIO(raw),names=True,dtype=None,encoding="utf8",delimiter=",")
    scores=[]
    fig,axes=plt.subplots(1,5,figsize=(14,3.8),sharey=True,layout="constrained")
    for ax,(trial,real,zero,end) in zip(axes,cases,strict=True):
        name=trial["name"]
        t,out,_,_,wheel=real
        keep=t<=end+1e-8
        ax.plot(t[keep]-t[0],wheel[keep].mean(axis=1),"k.-",label="SDK")
        for lead in [0,25,50,75,100]:
            run=data[(data["session"]==f"{name}_{lead}")&(data["variant"]=="canonical")]
            if not len(run):
                continue
            rt=run["t_s"]
            # Use the latest simulated report at or before each physical zero.
            idx=np.clip(np.searchsorted(rt,zero+1e-9,side="right")-1,0,len(run)-1)
            peak=np.max(np.abs([run["left_mmps"][idx],run["right_mmps"][idx]]))
            pos=np.interp([t[0],end],rt,run["out_mm"])
            physical=out[0]-np.interp(end,t,out)
            m=(run["left_mmps"]+run["right_mmps"])/2
            mae=np.mean(np.abs(np.interp(t[keep],rt,m)-wheel[keep].mean(axis=1)))
            scores.append([name,lead,len(zero),zero[-1]-zero[0],peak,physical,pos[0]-pos[1],mae])
            ax.plot(rt-t[0],m,lw=.9,label=f"{lead} ms lead")
        ax.set_title(f"Return {name}")
        ax.set_xlim(-.12,.5)
        ax.set_xlabel("Since last zero report (s)")
        ax.axvline(0,color="gray",ls=":",lw=.7)
    axes[0].set_ylabel("Mean reported wheel speed (mm/s)")
    axes[0].legend(fontsize=7)
    fig.savefig(ROOT/"mount-preload.png",dpi=160)
    with (ROOT/"mount-preload.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow("session lead_ms zero_samples idle_span_s max_sim_speed_at_zeros_mmps sdk_mm sim_mm mean_speed_mae_mmps".split())
        writer.writerows(scores)


if __name__=="__main__":
    main()

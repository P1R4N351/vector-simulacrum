"""Check the supervisor's known initial reverse command before pitch-gated steering."""
import csv
import io
import json
import os
import subprocess

import matplotlib.pyplot as plt
import numpy as np

from recorded_mount import ROOT, read_trial, states, table


def main():
    cases=[]
    inputs=[]
    for contact, observation in zip(table("return-scores.csv"),table("mount-target-inputs.csv"),strict=True):
        trial,real=read_trial(contact,observation)
        t,out,_,_,wheels=real
        rows,_=states(trial["name"])
        rows=[r for r in rows if t[0]<=float(r["receipt_s"])-contact["first_on_s"]<0]
        time=t[:len(rows)]
        head=np.array([float(r["head_rad"]) for r in rows])
        rate=-np.array([float(r["gy"]) for r in rows])
        pitch=np.rad2deg(np.r_[0,np.cumsum(.5*(rate[:-1]+rate[1:])*np.diff(time))]-(head-head[0]))
        # Stop at the last sampled state before the first estimated 2 degree threshold.
        stop=np.flatnonzero(pitch<=-2)[0]-1
        assert stop>1
        cases.append((trial,real,stop))
        inputs.append(dict(trial,name=trial["name"]+"reported",duration_s=t[stop]-t[0]+.01))
        for mode in ["last_zero","first_moving"]:
            samples=[[t[0],-30,-30]] if mode=="last_zero" else [[t[0],0,0],[t[1],-30,-30]]
            inputs.append(dict(trial,name=trial["name"]+mode,wheels=samples,immediate_wheels=True,
                               duration_s=t[stop]-t[0]+.01))
    raw=subprocess.check_output(["node",str(ROOT/"recorded_mount.mjs")],input=json.dumps(inputs),text=True,
                                env=dict(os.environ,VECTOR_TRACE_SHAFTS="1"))
    data=np.genfromtxt(io.StringIO(raw),names=True,dtype=None,encoding="utf8",delimiter=",")
    fig,axes=plt.subplots(2,5,figsize=(14,5.8),sharex="col",sharey="row",layout="constrained")
    scores=[]
    for col,(trial,real,stop) in enumerate(cases):
        t,out,_,_,wheels=real
        t=t[:stop+1]
        w=wheels[:stop+1].mean(axis=1)
        travel=out[0]-out[:stop+1]
        a,b=axes[:,col]
        a.plot(t-t[0],w,"k.-",label="SDK wheel mean")
        b.plot(t-t[0],travel,"k.-",label="SDK position")
        for mode,label in [("reported","Reported as command"),("last_zero","Start at last zero"),("first_moving","Start at first motion report")]:
            run=data[(data["session"]==trial["name"]+mode)&(data["variant"]=="canonical")]
            shaft=np.interp(t,run["t_s"],(run["shaft_l"]+run["shaft_r"])/2)
            pos=run["out_mm"][0]-np.interp(t,run["t_s"],run["out_mm"])
            a.plot(t-t[0],shaft,label=label)
            b.plot(t-t[0],pos,label=label)
            scores.append([trial["name"],mode,t[1]-t[0],t[-1]-t[0],travel[-1],pos[-1],np.median(w[2:]),
                           np.median(shaft[2:]),np.mean(np.abs(shaft-w))])
        a.axhline(-30,ls=":",lw=.7,color="gray")
        a.set_title(f"Return {trial['name']}")
        b.set_xlabel("Since last zero report (s)")
    axes[0,0].set_ylabel("Mean tread speed (mm/s)")
    axes[1,0].set_ylabel("Inward travel (mm)")
    axes[1,0].legend(fontsize=7)
    fig.savefig(ROOT/"mount-native-entry.png",dpi=160)
    with (ROOT/"mount-native-entry.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow("session onset first_report_gap_s window_s sdk_travel_mm sim_travel_mm median_sdk_mmps median_shaft_mmps speed_mae_mmps".split())
        writer.writerows(scores)


if __name__=="__main__":
    main()

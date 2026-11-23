"""Compare physical head-corrected pitch and fork travel with recorded mount replays."""
import csv
import io
import json
import os
import subprocess

import matplotlib.pyplot as plt
import numpy as np

from recorded_mount import ROOT, read_trial, states, table


def crossing(t, pitch):
    past = pitch <= -3
    ix = np.flatnonzero(past[:-1] & past[1:])
    return t[ix[0]] if len(ix) else np.nan


def main():
    contacts = table("return-scores.csv")
    cases = [read_trial(c,o) for c,o in zip(contacts,table("mount-target-inputs.csv"),strict=True)]
    trials = [trial for trial,_ in cases]
    raw = subprocess.check_output(["node",str(ROOT/"recorded_mount.mjs")],text=True,input=json.dumps(trials),
                                  env=dict(os.environ,VECTOR_TRACE_DOCK_PITCH="1"))
    sim = np.genfromtxt(io.StringIO(raw),names=True,delimiter=",",dtype=None,encoding="utf8")
    optical = table("return-frames.csv")
    fig, axes = plt.subplots(2,5,figsize=(14,6),sharey="row",layout="constrained")
    scores=[]
    for col,(c,(trial,real)) in enumerate(zip(contacts,cases,strict=True)):
        name=trial["name"]
        rows,_=states(name)
        rows=[r for r in rows if trial["wheels"][0][0] <= float(r["receipt_s"])-c["first_on_s"] <= .5]
        t=np.array([float(r["receipt_s"])-c["first_on_s"] for r in rows])
        def channel(key):
            return np.array([float(r[key]) for r in rows])
        rate=-channel("gy")
        head=channel("head_rad")
        pitch=np.rad2deg(np.r_[0,np.cumsum(.5*(rate[:-1]+rate[1:])*np.diff(t))] - (head-head[0]))
        even=np.linspace(t[0],t[-1],len(t))
        uniform=np.rad2deg(np.r_[0,np.cumsum(.5*(rate[:-1]+rate[1:])*np.diff(even))] - (head-head[0]))
        # Compare apparent gravity only for reports where the wheels have stopped.
        gravity=np.rad2deg(np.arctan2(channel("ax"),channel("az"))-head)
        still=(channel("left_mmps")==0)&(channel("right_mmps")==0)
        rest=still&(t>.2)&(t<.5)
        grav_delta=np.median(gravity[rest])-gravity[0] if np.any(rest) else np.nan
        a,b=axes[:,col]
        a.plot(t,pitch,color="black",label="Head-corrected gyro")
        a.plot(even,uniform,"--",color="gray",label="Even SDK cadence")
        cam=optical[(optical["session"]==name)&(optical["rel_s"]<0)]
        ct=cam["rel_s"]
        travel=cam["forward_mm"][0]-cam["forward_mm"]
        b.plot(ct,travel,"ko-",label="Camera fork")
        for variant,label in [("canonical","Canonical"),("driven_rear_envelope","Driven support")]:
            run=sim[(sim["session"]==name)&(sim["variant"]==variant)]
            st=run["t_s"]
            angle=np.interp(t,st,run["pitch_deg"])-run["pitch_deg"][0]
            pos=np.interp(ct,st,run["out_mm"])
            a.plot(t,angle,label=label)
            b.plot(ct,pos[0]-pos,".-",label=label)
            j=np.flatnonzero(t<0)[-1]
            scores.append([name,variant,crossing(t,pitch),crossing(even,uniform),crossing(t,angle),
                           pitch[j],angle[j],np.median(pitch[rest]) if np.any(rest) else np.nan,
                           grav_delta, travel[-1],pos[0]-pos[-1], np.interp(crossing(t,pitch),real[0],real[1]),
                           np.interp(crossing(t,angle),st,run["out_mm"])])
        a.axhline(-3,color="gray",ls=":",lw=.7)
        a.set_title(f"Return {name}")
        for ax in (a,b):
            ax.set_xlabel("From first on-report (s)")
    axes[0,0].set_ylabel("Pitch change (°, nose up positive)")
    axes[1,0].set_ylabel("Inward travel (mm)")
    axes[0,0].legend(fontsize=7)
    axes[1,0].legend(fontsize=7)
    fig.savefig(ROOT/"mount-pitch.png",dpi=160)
    with (ROOT/"mount-pitch.csv").open("w") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow("session model gyro_cross_s even_cross_s sim_cross_s gyro_last_off_deg sim_last_off_deg gyro_rest_deg gravity_rest_deg camera_travel_mm sim_travel_mm sdk_out_at_cross_mm sim_out_at_cross_mm".split())
        writer.writerows(scores)


if __name__=="__main__":
    main()

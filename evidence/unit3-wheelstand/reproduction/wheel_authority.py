"""Measure pitch conditional on encoder speed tracking, with bounded virtual torque."""
import csv
import sys
from unittest.mock import patch

import matplotlib.pyplot as plt
import numpy as np

from encoder_follow import evaluate, here, replay
from wheel_follow import trial


def servo(model,v,tr,start,limit,gain):
    control = replay._Run.control
    radius = v["wheel_front_radius_mm"]
    records = []
    for side in ("left","right"):
        a = model.actuator(side+"_wheel")
        a.gainprm[0] = gain
        a.biasprm[2] = -gain
        a.ctrlrange[:] = [-100,100]
        a.forcelimited[0] = 1
        a.forcerange[:] = [-limit,limit]

    def command(r,t):
        control(r,t)
        values = []
        for side in ("left","right"):
            a = r.d.actuator(side+"_wheel")
            a.ctrl[0] = tr.obs(t,side)/radius
            values.append(r.d.joint("wheel_f"+side[0]+"_joint").qvel[0]*radius)
            values.append(float(a.force[0])*1000)
        records.append([t,*values])
    with patch.object(replay._Run,"control",command):
        trace,score = evaluate(model,v,tr,start,tr.late,True)
    return trace,np.array(records),score


def main(fixture):
    v = replay.params.values(replay.params.load(model="vector_2"),{"tip_supports_driven":1})
    rows = []
    fig,axes = plt.subplots(2,2,figsize=(10,6),sharex=True,sharey="row",layout="constrained")
    for col,name in enumerate(("W4","W8")):
        tr = replay.Trial(name)
        t = tr.state["sec"]-tr.zero
        mask = (t>=tr.late)&(t<=tr.late+.8)
        obs = np.column_stack([tr.state[s][mask] for s in ("left","right")])
        for ax,y in zip(axes[:,col],(obs.mean(axis=1),np.degrees(tr.state["pitch"][mask]))):
            ax.plot(t[mask]-tr.late,y,"ko-",ms=3,label="Physical SDK",zorder=4)
        for i,mode in enumerate(("Original","Low gain","High gain","High gain · 4× cap")):
            for start in (3.5,3.8):
                model = replay.build(v,fixture)
                model.opt.timestep = .00025
                a = model.actuator("lift")
                a.gainprm[0] = 5
                a.biasprm[1:3] = [-5,-.03]
                stall = float(model.actuator("left_wheel").gainprm[0])
                if i:
                    limit = stall*(4 if i==3 else 1)
                    trace,w,score = servo(model,v,tr,start,limit,.015 if i==1 else 1.5)
                    in_window = (w[:,0]>=tr.late)&(w[:,0]<=tr.late+.8)
                    torque = abs(w[in_window][:,[2,4]])
                    peak = torque.max()
                    fraction = (torque>=limit*999).mean()
                    speeds = w[:,[1,3]]
                else:
                    trace,w,score = trial(model,v,tr,start,tr.late,True)
                    speeds = w[:,1:]
                    peak = fraction = limit = None
                at_sdk = np.column_stack([np.interp(t[mask],w[:,0],s) for s in speeds.T])
                rms = np.mean((at_sdk-obs)**2)**.5
                rows.append([name,start,mode,stall*1000,limit*1000 if i else None,peak,fraction,rms,*score[:3]])
                for ax,clock,y in zip(axes[:,col],(w[:,0],trace[:,0]),(speeds.mean(axis=1),trace[:,1])):
                    ax.plot(clock-tr.late,y,color=f"C{i}",alpha=.7,label=mode if start==3.5 else None)
        axes[0,col].set_title(name)
        axes[1,col].set(xlim=(-.04,.8),xlabel="Seconds from last raised-lift sample")
    axes[0,0].set_ylabel("Mean wheel speed (mm/s)")
    axes[1,0].set_ylabel("Chassis pitch (°)")
    axes[0,0].legend(fontsize=7)
    fig.savefig(here/"wheel-authority.png",dpi=160)
    with (here/"wheel-authority.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("run release_W4_clock mode stall_mNm cap_mNm peak_sampled_mNm fraction_at_cap wheel_rms_mmps lift_rms_mm peak_pitch_deg held_pitch_deg".split())
        w.writerows(rows)


if __name__=="__main__": main(sys.argv[1])

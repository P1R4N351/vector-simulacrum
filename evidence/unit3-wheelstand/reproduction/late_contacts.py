"""Inspect cube contact loading and test an earlier wheelstand brake on held-out W8."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import mujoco
import numpy as np

import replay

HERE = Path(__file__).resolve().parent
KINDS = ("shelf", "fork", "other_robot")


def wheel_fall(trial):
    states = trial.state
    t = states["sec"] - trial.zero - trial.late
    speed = (states["left"] + states["right"]) / 2
    for i in np.flatnonzero((t[:-1] >= 0) & (t[:-1] < .5) & (speed[:-1] > 70) & (speed[1:] <= 70)):
        return float(t[i] + (t[i+1]-t[i]) * (70-speed[i])/(speed[i+1]-speed[i]))
    raise ValueError("No post-descent falling crossing")


class LoadTrace:
    def __init__(self, model):
        self.rows = []
        bodies = model.geom_bodyid
        self.cube = bodies == model.body("cube").id
        self.category = np.where(bodies == 0, 0, np.where(bodies == model.body("lift_fork").id, 1, 2))
        self.wrench = np.zeros(6)
        self.other_geom = []

    def step(self, r, t):
        loads = [0., 0., 0.]
        largest_other = 0.
        other_name = ""
        for i, c in enumerate(r.d.contact):
            a,b = c.geom
            if self.cube[a] == self.cube[b]:
                continue
            mujoco.mj_contactForce(r.m,r.d,i,self.wrench)
            other = b if self.cube[a] else a
            category = self.category[other]
            normal = max(0.,self.wrench[0])
            loads[category] += normal
            if category == 2 and normal > largest_other:
                largest_other = normal
                other_name = r.m.geom(other).name
        self.other_geom.append(other_name)
        self.rows.append([t,*loads,r.d.sensor("cube_accel").data[2]/(r.v["gravity_mmps2"]*.001)])


def main():
    w4 = replay.Trial("W4")
    brake_offset = wheel_fall(w4)
    fig, axes = plt.subplots(3,2,figsize=(11,8),layout="constrained",sharex="col")
    with (HERE/"late-contact-cases.csv").open("w",newline="") as cf, (HERE/"late-contact-loads.csv").open("w",newline="") as lf:
        cases, loads = (csv.writer(f,lineterminator="\n") for f in (cf,lf))
        cases.writerow(["run","case","drive","release_s","onset_s","empirical_stop_s","last_recorded_packet_change_s",
                        "packet_changes","late_changes","peak_pitch_deg","held_pitch_deg","peak_g"])
        loads.writerow(["run","case","drive","packet_s","after_last_recorded_change","peak_shelf_N","peak_fork_N","peak_other_robot_N","peak_abs_chip_z_g","other_geom_at_peak"])
        for col,name in enumerate(("W4","W8")):
            tr = replay.Trial(name)
            v = replay.params.values(replay.params.load(model="vector_2"),{"tip_supports_driven":1})
            m = replay.build(v,sys.argv[1])
            m.opt.timestep = v["sim_timestep_ms"]*.001/4
            packets = tr.cube[::3]
            t = packets["sec"]-tr.zero
            lo,hi = tr.late-.3,min(t[-1],tr.late+1.2)
            real = t[1:][np.diff(packets["tap"])!=0]
            real = real[(real>=lo)&(real<=hi)]
            sdk_time = tr.state["sec"]-tr.zero
            axes[0,col].plot(sdk_time,np.rad2deg(tr.state["pitch"]),".-",color="black",label="Physical SDK")
            # The three retained accelerometer slots share one host receipt.
            all_t = tr.cube["sec"]-tr.zero
            axes[2,col].scatter(all_t,tr.cube["z"]/8192,color="black",s=3,label="Physical slots")
            for case,(start,onset) in enumerate(((s+tr.shift,o) for s in (3.5,3.8) for o in (tr.early,tr.late)),1):
                chassis = m.geom("chassis_geom")
                front_size, center = chassis.size[0], chassis.pos[0]
                for drive,stop in (("stock replay",None),("early brake",tr.late+brake_offset),("front inset 2 mm",None)):
                    # Inset the front face only. The explicit inertial stays fixed.
                    delta = .001 if drive == "front inset 2 mm" else 0.
                    chassis.size[0], chassis.pos[0] = front_size-delta, center-delta
                    monitor = LoadTrace(m)
                    events = []
                    trace,pk,n,_ = replay.run(m,v,tr,start,onset,"servo:60",17,tap_times=events,on_step=monitor.step,stop_s=stop)
                    metrics = replay.score(tr,trace,pk,n,onset)
                    events=[e for e in events if lo<=e<=hi]
                    data=np.array(monitor.rows)
                    late=lambda e: e>real[-1]+.060
                    cases.writerow([name,case,drive,start,onset,stop,real[-1],len(events),sum(map(late,events)),
                                    metrics["peak_pitch_deg"],metrics["hold_pitch_deg"],metrics["peak_g"]])
                    for event in events:
                        indices=np.flatnonzero((data[:,0]>event-.03)&(data[:,0]<=event+1e-8))
                        win=data[indices]
                        geom=monitor.other_geom[indices[np.argmax(win[:,3])]]
                        loads.writerow([name,case,drive,event,int(late(event)),*np.max(abs(win[:,1:]),axis=0),geom])
                    if case==1:
                        axes[0,col].plot(trace[:,0],trace[:,1],label=drive)
                        if drive == "stock replay":
                            for i,kind in enumerate(KINDS):
                                axes[1,col].plot(data[:,0],data[:,i+1],lw=.7,label=kind)
                            axes[2,col].plot(data[:,0],data[:,4],lw=.65,alpha=.65,label="Simulated chip force")
                            for e in events:axes[1,col].axvline(e,ls=":",color=".35",lw=1)
                chassis.size[0], chassis.pos[0] = front_size, center
            for ax in axes[:,col]:
                ax.set_xlim(lo,hi)
                ax.axvline(real[-1],color="#97510a",lw=1,ls="--")
            axes[0,col].set_title(name+" · early release / early descent")
            axes[2,col].set_xlabel("Seconds after wheelstand command")
    for ax,label in zip(axes[:,0],["Chassis pitch (°)","Cube normal load (N)","Chip Z acceleration (g)"]):
        ax.set_ylabel(label);ax.legend(fontsize=8)
    fig.savefig(HERE/"late-contacts.png",dpi=160)

if __name__=="__main__":
    main()

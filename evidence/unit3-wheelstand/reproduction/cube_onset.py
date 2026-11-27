"""Compare measured and replayed cube response timestamps using retained 3-axis slots."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import mujoco
import numpy as np

import replay

HERE = Path(__file__).resolve().parent
THRESHOLDS = (.1,.2,.5,1.0)


def envelope(slots, rest):
    return np.linalg.norm(slots-rest,axis=-1).max(axis=1)


def first(t, level, threshold, lo, hi):
    hits = np.flatnonzero((t>=lo)&(t<=hi)&(level>=threshold))
    return t[hits[0]] if len(hits) else np.nan


def main(fixture):
    v = replay.params.values(replay.params.load(model="vector_2"),{"tip_supports_driven":1})
    m = replay.build(v,fixture)
    m.opt.timestep = v["sim_timestep_ms"]*.001/4
    fig,axes = plt.subplots(2,2,figsize=(11,6),layout="constrained",sharex="col",sharey=True)
    with (HERE/"cube-onset.csv").open("w",newline="") as f:
        w = csv.writer(f,lineterminator="\n")
        w.writerow(["run","pose","timing_case","threshold_g","recorded_receipt_s","sim_packet_s","sim_minus_recorded_ms","first_robot_contact_s","first_robot_geom","initial_overlap_mm"])
        for col,name in enumerate(("W4","W8")):
            tr = replay.Trial(name)
            c = tr.cube
            assert np.array_equal(c["slot"],np.tile(np.arange(3),len(c)//3))
            t = c["sec"][::3]-tr.zero
            xyz = np.stack([c[k] for k in "xyz"],axis=1).reshape(-1,3,3)/8192
            rest = np.median(xyz[(t>=-.8)&(t<0)].reshape(-1,3),axis=0)
            measured = envelope(xyz,rest)
            lo,hi = tr.late-.20,tr.late+.36
            nominal = tr.cube_xy.copy()
            offsets = [("nominal",0.)]
            if name=="W8":
                with (tr.dir/"camera-pose.csv").open() as camera_file:
                    rows = list(csv.DictReader(camera_file))
                center = next(r for r in rows if r["case"]=="geometry" and [float(r[k]) for k in ("focal_px","lens_height_mm","fork_height_mm")]==[498.4,65.5,93])
                offsets.append(("forward only",max(float(r["forward_mm"]) for r in rows)-float(center["forward_mm"])))
                offsets.append(("near forward only",min(float(r["forward_mm"]) for r in rows)-float(center["forward_mm"])))
            for row,(label,offset) in enumerate(offsets):
                yaw = tr.obs(3.065,"yaw")
                tr.cube_xy = nominal+offset*np.array([np.cos(yaw),np.sin(yaw)])
                ax = axes[0,col] if row==0 else axes[1,row-1]
                ax.plot(t-tr.late,measured,"ko-",ms=3,lw=1.2,label="Physical receipts")
                for case,(s,o) in enumerate(((s,o) for s in (3.5,3.8) for o in (tr.early,tr.late)),1):
                    start = s+tr.shift
                    holder = []
                    contact = []
                    wrench = np.empty(6)
                    def keep(run,_t):
                        if not holder:
                            holder.append(run.sensor)
                        if contact:
                            return
                        for i,candidate in enumerate(run.d.contact):
                            a,b = candidate.geom
                            if m.geom_bodyid[a]==run.cube_id:
                                other = b
                            elif m.geom_bodyid[b]==run.cube_id:
                                other = a
                            else:
                                continue
                            if m.geom_bodyid[other]==0:
                                continue
                            mujoco.mj_contactForce(m,run.d,i,wrench)
                            if wrench[0]>=.01:
                                contact.extend((_t,m.geom(other).name))
                                break
                    _, _, _, overlap = replay.run(m,v,tr,start,o,"servo:60",17,on_step=keep)
                    messages = holder[0].messages
                    st = np.array([start+ts for ts,_ in messages])
                    slots = np.stack([np.frombuffer(p[2:],"<i2").reshape(3,3) for _,p in messages])/8192
                    baseline = np.array([0.,0.,1.])*np.array(v["cube_accel_scale"])+np.array(v["cube_accel_bias_mg"])*.001
                    sim = envelope(slots,baseline)
                    ax.plot(st-tr.late,sim,".-",ms=3,lw=.8,label=f"Replay {case}")
                    for threshold in THRESHOLDS:
                        rt = first(t,measured,threshold,lo,hi)
                        simt = first(st,sim,threshold,lo,hi)
                        w.writerow([name,label,case,threshold,rt,simt,(simt-rt)*1000,*(contact or [np.nan,""]),overlap])
                ax.set_title(name+" · "+label)
                ax.set_xlim(-.20,.36)
                ax.grid(alpha=.22)

    axes[0,0].legend(fontsize=7)
    axes[0,0].set_ylabel("Peak slot deviation from rest (g)")
    axes[1,0].set_xlabel("Seconds from last raised-lift sample")
    axes[1,1].set_xlabel("Seconds from last raised-lift sample")
    fig.savefig(HERE/"cube-onset.png",dpi=160)


if __name__=="__main__":
    main(sys.argv[1])

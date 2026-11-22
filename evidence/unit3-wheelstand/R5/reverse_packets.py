"""Use R5 wheel speeds to predict carried-cube packets across four reverse commands."""
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

here = Path(__file__).resolve().parent
prior = here.parents[1]/"unit3-cube-accelerometer"
sys.path.insert(0, str(prior))
from replay_turns import predicted, values


def read(path):
    return np.genfromtxt(path, names=True, delimiter=",")


def rms(delta):
    return np.sqrt(np.mean(np.sum(delta**2, axis=1)))



def score_lag(t,obs,starts,ends,rt,raw,stationary,uptime,reference):
    """Positive delay moves a simulated event later on the receipt clock."""
    delays = np.arange(-300,301,5)
    errors = np.empty((4,len(delays)))
    masks = [(t>=lo)&(t<hi) for lo,hi in zip(starts,ends)]
    for i,mask in enumerate(masks):
        assert t[mask].min()-.3 >= rt.min() and t[mask].max()+.3 <= rt.max()
        for j,ms in enumerate(delays):
            p = np.column_stack([np.interp(t[mask]-ms/1000,rt,col) for col in raw.T])
            errors[i,j] = rms(p-obs[mask])
    counts = np.array([np.sum(m) for m in masks])
    pooled = np.sqrt(np.average(errors[:2]**2,axis=0,weights=counts[:2]))
    selected = np.argmin(pooled)
    zero = np.flatnonzero(delays==0)[0]
    # Subtract a reference offset to quantify delivery jitter independently of the replay.
    excess = (t-uptime)-np.median((t-uptime)[reference])
    with (here/"reverse-lag.csv").open("w") as f:
        w = csv.writer(f,lineterminator="\n")
        w.writerow(["move","packets","trained_delay_ms","unshifted_rms_g","shifted_rms_g",
                    "constant_rms_g","best_individual_delay_ms","best_individual_rms_g",
                    "receipt_minus_uptime_p05_ms","receipt_minus_uptime_median_ms","receipt_minus_uptime_p95_ms"])
        for i,mask in enumerate(masks):
            j = np.argmin(errors[i])
            w.writerow([i+1,counts[i],delays[selected],errors[i,zero],errors[i,selected],
                        rms(obs[mask]-stationary),delays[j],errors[i,j],
                        *np.round(np.percentile(excess[mask]*1000,[5,50,95]),2)])
    fig,axes = plt.subplots(2,2,figsize=(10,6),layout="constrained")
    for i,(ax,mask) in enumerate(zip(axes.flat,masks)):
        ax.plot(delays,errors[i],label="Rigid carry")
        ax.axhline(rms(obs[mask]-stationary),color=".4",ls=":",label="Pre-reverse mean")
        ax.axvline(delays[selected],color="C1",ls="--",label="Delay from moves 1–2")
        ax.set(title=f"Reverse {i+1} · "+("training" if i<2 else "held out"),
               xlabel="Delay added to replay (ms)",ylabel="Vector RMS error (g)")
    axes[0,0].legend(fontsize=8)
    fig.savefig(here/"reverse-lag.png",dpi=150)
    plt.close(fig)


def main():
    with (here/"actions.csv").open() as f:
        actions = [r for r in csv.DictReader(f) if r["action"] == "reverse inward"]
    starts, ends = [np.array([float(r["sec"]) for r in actions if r["kind"] == k])
                    for k in ("begin","end")]
    c = read(here/"cube.csv.gz")
    assert np.array_equal(c["slot"].reshape(-1,3), np.tile([0,1,2],(len(c)//3,1)))
    t = c["sec"][::3]
    packets = np.column_stack([c[k] for k in "xyz"]).reshape(-1,3,3)
    keep = np.r_[True,np.any(np.diff(packets,axis=0)!=0,axis=(1,2))]
    t, obs = t[keep], packets[keep].mean(axis=1)/8192
    held = (t>=29)&(t<31.5)
    stationary = obs[held].mean(axis=0)
    up = stationary/np.linalg.norm(stationary)
    gravity_pose = Rotation.from_quat(np.r_[np.cross(up,[0,0,1]),1+up[2]])
    robot = read(here/"state.csv.gz")
    robot = robot[(robot["sec"]>=29)&(robot["sec"]<ends[-1]+.5)]
    path = np.rec.fromarrays([robot[k] for k in ("head","lift","left","right")],
                             names="head_rad,lift_mm,left_mmps,right_mmps")
    train = (t>=starts[0])&(t<ends[1])
    def simulate(yaw, z_gain):
        q = (Rotation.from_euler("z",yaw,degrees=True)*gravity_pose).as_quat(scalar_first=True)
        rt, a = predicted(robot["sec"]-starts[0],path,q,dict(values,cube_accel_scale=[1,1,z_gain]))
        return rt+starts[0], a
    def at_receipt(rt, samples):
        return np.column_stack([np.interp(t,rt,k) for k in samples.T])
    native = {yaw:simulate(yaw,1.) for yaw in range(0,360,90)}
    nominal = {yaw:at_receipt(*trace) for yaw,trace in native.items()}
    best = min(nominal,key=lambda yaw:rms((nominal[yaw]-obs)[train]))
    rt, raw = native[best]
    score_lag(t,obs,starts,ends,rt,raw,stationary,c["uptime"][::3][keep],held)
    rest = read(prior/"resting.csv")
    mean = np.array([rest[k+"_counts"].mean()/8192 for k in "xyz"])
    gain = mean[2]/np.sqrt(1-mean[:2]@mean[:2])
    candidates = {"Pre-reverse mean":np.tile(stationary,(len(t),1)),
                  "Nominal":nominal[best], "Earlier Z gain":at_receipt(*simulate(best,gain))}
    fig,axes = plt.subplots(3,1,sharex=True,figsize=(9,6),layout="constrained")
    view = (t>=starts[0]-.25)&(t<=ends[-1])
    for i,ax in enumerate(axes):
        ax.plot(t[view]-starts[0],obs[view,i],"k.",ms=3,label="Recorded packet mean")
        for label in ("Nominal","Earlier Z gain"):
            ax.plot(t[view]-starts[0],candidates[label][view,i],lw=1,label=label)
        ax.axvspan(starts[2]-starts[0],ends[-1]-starts[0],color="gray",alpha=.12,label="Held out")
        ax.set_ylabel("XYZ"[i]+" (g)")
    axes[0].legend(ncol=4,fontsize=8)
    axes[-1].set_xlabel("Seconds from first reverse command")
    fig.savefig(here/"reverse-packets.png",dpi=150)
    with (here/"reverse-packets.csv").open("w") as f:
        w = csv.writer(f,lineterminator="\n")
        w.writerow(["move","held_out","model","yaw_deg","packets","vector_rms_error_g","z_mean_error_g","measured_vector_scatter_g","predicted_vector_scatter_g","demeaned_error_g"])
        for i,(lo,hi) in enumerate(zip(starts,ends)):
            mask = (t>=lo)&(t<hi)
            for label,p in candidates.items():
                e = (p-obs)[mask]
                w.writerow([i+1,int(i>=2),label,best if label!="Pre-reverse mean" else "",len(e),round(rms(e),6),round(e.mean(axis=0)[2],6),
                            round(rms(obs[mask]-obs[mask].mean(0)),6),round(rms(p[mask]-p[mask].mean(0)),6),
                            round(rms(e-e.mean(0)),6)])


if __name__ == "__main__":
    main()

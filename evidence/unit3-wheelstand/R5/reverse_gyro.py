"""Compare the R5 head gyro with the same wheel-driven rigid-carry replay."""
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[1]/"unit3-native-pickup"))
from replay_carry import fixture, run, values
from simulacrum.imu_filter import ImuFilter


def main():
    with (here/"actions.csv").open() as f:
        moves = [r for r in csv.DictReader(f) if r["action"]=="reverse inward"]
    starts, ends = [np.array([float(r["sec"]) for r in moves if r["kind"]==k])
                    for k in ("begin","end")]
    r = np.genfromtxt(here/"state.csv.gz",delimiter=",",names=True)
    r = r[(r["sec"]>=29)&(r["sec"]<ends[-1]+.5)]
    t = r["sec"]-starts[0]
    measured = np.degrees(np.column_stack([r[k] for k in ("gx","gy","gz")]))
    c = np.genfromtxt(here/"carry-attitude.csv",delimiter=",",names=True,
                      dtype=None,encoding="utf8")[0]
    up = np.array([c["mean_"+k+"_g"] for k in "xyz"])
    up /= np.linalg.norm(up)
    tilt = Rotation.from_quat(np.r_[np.cross(up,[0,0,1]),1+up[2]])
    pose = (Rotation.from_euler("z",180,degrees=True)*tilt).as_quat(scalar_first=True)
    def configure(root):
        fixture(root, True)
        root.find(".//body[@name='cube']").set("quat"," ".join(map(str,pose)))
    filt = ImuFilter(values["accel_filter_cutoff_hz"],values["gyro_filter_cutoff_hz"],
                     values["imu_filter_second_pole"],values["sim_timestep_ms"]/1000)
    dense = []
    def observe(m,d,clock):
        imu = np.r_[d.sensor("accel").data,d.sensor("gyro").data]
        if not dense:
            filt.reset(imu)
        else:
            filt.update(imu)
        dense.append([clock,*np.degrees(filt.output[3:])])
    run([{"head":r["head"][0],"lift":r["lift"][0]}],t,
        np.column_stack([r["left"],r["right"]]),40.5,configure=configure,observe=observe)
    dense = np.array(dense)
    predicted = np.column_stack([np.interp(t,dense[:,0],v) for v in dense[:,1:].T])
    # Remove each channel's pre-motion offset, retaining transient and mean motion.
    pre = (r["sec"]>=29.5)&(r["sec"]<31.5)
    measured -= measured[pre].mean(axis=0)
    predicted -= predicted[pre].mean(axis=0)
    fig,axes = plt.subplots(4,1,sharex=True,figsize=(9,7),layout="constrained")
    for j,ax in enumerate(axes[:3]):
        ax.plot(t,measured[:,j],"k.-",ms=3,lw=.8,label="SDK head gyro")
        ax.plot(t,predicted[:,j],lw=1,label="Simulated head gyro")
        ax.set_ylabel("XYZ"[j]+" (°/s)")
    axes[0].legend()
    axes[3].plot(t,np.degrees(r["head"]-r["head"][pre].mean()),label="Head encoder")
    axes[3].plot(t,np.degrees(r["pitch"]-r["pitch"][pre].mean()),label="SDK chassis pitch")
    axes[3].set(ylabel="Change (°)",xlabel="Seconds from first reverse command",xlim=(-.3,ends[-1]-starts[0]+.15))
    axes[3].legend()
    for ax in axes:
        ax.axvspan(starts[2]-starts[0],ends[-1]-starts[0],color=".6",alpha=.15)
    fig.savefig(here/"reverse-gyro.png",dpi=150)
    with (here/"reverse-gyro.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["move","axis","sdk_samples","measured_rms_dps","sim_rms_dps","error_rms_dps","correlation",
                    "head_range_deg","sdk_pitch_range_deg","lift_range_mm","rest_measured_rms_dps"])
        for move,(lo,hi) in enumerate(zip(starts,ends),1):
            use = (r["sec"]>=lo)&(r["sec"]<hi)
            for j,k in enumerate("XYZ"):
                obs,sim = measured[use,j],predicted[use,j]
                w.writerow([move,k,len(obs),*[round(np.sqrt(np.mean(x*x)),4) for x in (obs,sim,sim-obs)],
                            round(np.corrcoef(obs,sim)[0,1],4),round(np.degrees(np.ptp(r["head"][use])),3),
                            round(np.degrees(np.ptp(r["pitch"][use])),3),round(np.ptp(r["lift"][use]),3),
                            round(np.sqrt(np.mean(measured[pre,j]**2)),4)])


if __name__ == "__main__":
    main()

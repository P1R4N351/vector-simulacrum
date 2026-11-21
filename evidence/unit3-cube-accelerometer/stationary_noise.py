"""Estimate stationary packet noise on one capture and score two untouched captures."""
from pathlib import Path
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt

from decode import decode

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[1]))
from simulacrum import params
from simulacrum.cube import CubeAccelerometer

v = params.values(params.load(model="vector_2"))
step = v["sim_timestep_ms"]/1000
nominal = 220.  # nominal comparator fixed before calibration


def scatter(slots):
    # Pair differences within a packet avoid long receiver gaps and slow DC drift.
    return 1000*np.sqrt(np.mean(np.diff(slots, axis=1)**2, axis=(0,1))/2)


def observations(name):
    packets,_ = decode(here/(name+".tsv"))
    if name == "pickup":
        # Last second before pickup, matching the published stationary comparison.
        with (here/"action-times.csv").open() as f:
            zero=float(next(r["pod_unix_s"] for r in csv.DictReader(f) if r["action"]=="pickup"))
        packets=[p for p in packets if zero-1 <= p[0]*1e-9 < zero]
    return np.stack([np.frombuffer(p[3],dtype="<i2").reshape(3,3) for p in packets])/8192


def simulate(density, force, seed):
    chip=CubeAccelerometer(dict(v,cube_accel_noise_density_ug=density),step,seed=seed)
    chip.reset(force,0.)
    for n in range(1,round(20/step)+1):
        chip.step(n*step,force)
    return np.stack([np.frombuffer(p[2:],dtype="<i2").reshape(3,3) for _,p in chip.messages])/8192


def main():
    fit=observations("resting")
    force=fit.mean(axis=(0,1))
    real=np.linalg.norm(scatter(fit))
    predicted=np.linalg.norm(scatter(simulate(nominal,force,0)))
    density=round(nominal*real/predicted)
    comparison=[("Nominal",nominal),("Stationary fit",density)]
    rows=[]
    fig,axes=plt.subplots(1,3,figsize=(10,3.5),sharey=True)
    for ax,(n,name) in zip(axes,enumerate(["resting","undock-check","pickup"])):
        data=observations(name)
        measured=scatter(data)
        ax.bar(np.arange(3)-.25,measured,.25,label="Recorded")
        for j,(label,d) in enumerate(comparison):
            modeled=scatter(simulate(d,force,n+1))
            err=np.sqrt(np.mean((measured-modeled)**2))
            rows.append([name,len(data),label,d,*measured,*modeled,err])
            ax.bar(np.arange(3)+j*.25,modeled,.25,label=label)
        ax.set(xticks=range(3),xticklabels=list("XYZ"),title=name)
    axes[0].set_ylabel("Within-packet noise (mg RMS)")
    axes[-1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(here/"stationary-noise.png",dpi=150)
    with (here/"stationary-noise.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["capture","packets","model","density_ug_sqrtHz","recorded_x_mg","recorded_y_mg",
                    "recorded_z_mg","model_x_mg","model_y_mg","model_z_mg","axis_rms_error_mg"])
        w.writerows(rows)
    for r in rows:print(r[:4],r[-1])

if __name__=="__main__":
    main()

"""Separate stationary-cube approach dynamics from initial registration and contact."""
from pathlib import Path
import csv
import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
def load(path):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf8")

ranges = load(HERE/"samples.csv")
poses = load(HERE.parent/"unit3-cube-contact-replay/inputs.csv")
overhead = load(HERE.parent/"unit3-paired-cube-push/traces.csv")
cases = []
for trial in range(1, 6):
    r = ranges[(ranges["trial"] == trial) & (ranges["seconds"] <= 4.25)]
    p = poses[poses["trial"] == trial]
    h = np.deg2rad(p["yaw_deg"][0])
    travel = (p["x_mm"]-p["x_mm"][0])*np.cos(h) + (p["y_mm"]-p["y_mm"][0])*np.sin(h)
    quiet = r["seconds"] < 1.5
    initial = np.median(r["real_mm"][quiet])
    windows = (r["seconds"] >= 2.5) & (r["real_found"] == 1)
    o = overhead[(overhead["trial"] == trial) & (overhead["camera"] == "frames")
                 & (overhead["seconds"] >= 0) & (overhead["seconds"] <= 2.75)]
    assert len(o) and np.max(np.hypot(o["forward_mm"],o["left_mm"])) < 1
    cases.append((r, p, travel, initial, windows))

def prediction(case, gain, lag):
    r,p,travel,initial,_ = case
    movement = np.interp(r["seconds"]-lag,p["sec"],travel)
    return np.maximum(30, initial - gain*movement)

def residual(cases, gain, lag):
    return np.concatenate([(prediction(c,gain,lag)-c[0]["real_mm"])[c[-1]] for c in cases])

if __name__ == "__main__":
    train = cases[:2]
    lag = least_squares(lambda x: residual(train,1,x[0]), [0.1], bounds=(0,.75)).x[0]
    gain = least_squares(lambda x: residual(train,x[0],0), [1.],bounds=(.1,2.)).x[0]
    joint = least_squares(lambda x: residual(train,*x), [.9,.1],bounds=([.1,0],[2,.75])).x
    models = {"Unit gain": (1,0), "Lag alone": (1,lag), "Gain alone": (gain,0), "Gain and lag": joint}
    print(models)
    fig, axes=plt.subplots(1,5,figsize=(13,3.3),sharey=True)
    with (HERE/"approach-scores.csv").open("w") as out:
        writer=csv.writer(out,lineterminator="\n")
        writer.writerow(["trial","model","gain","lag_s","rows","mae_mm","bias_mm","within_5pct"])
        for trial,(case,ax) in enumerate(zip(cases,axes),1):
            r,p,travel,initial,selection=case
            t=r["seconds"]-1.5
            ax.plot(t,r["real_mm"],"k.",ms=3,label="Recorded")
            for label, (g,d) in models.items():
                pred=prediction(case,g,d)
                err=(pred-r["real_mm"])[selection]
                writer.writerow([trial,label,g,d,len(err),np.mean(abs(err)),np.mean(err),sum(abs(err)<=.05*r["real_mm"][selection])])
                if label in ("Unit gain","Gain and lag"):
                    ax.plot(t,pred,label=label)
            ax.axvspan(1,2.75,alpha=.08,color="k")
            ax.set(title=f"Trial {trial} · "+("fit" if trial<3 else "held out"), xlabel="Approach time (s)")
    axes[0].set_ylabel("Distance (mm)")
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(HERE/"approach.png",dpi=160)

    # Compare independent motion estimates at identical image times, using the
    # earlier optical-flow scale and timing fit frozen on trials 1 and 2.
    edges = load(HERE.parent/"unit3-cube-camera-scale/edges.csv")
    with (HERE/"approach-optical.csv").open("w") as out:
        writer = csv.writer(out,lineterminator="\n")
        writer.writerow(["trial","image_receipt_s","odometry_mm","optical_mm","range_drop_mm"])
        for trial,case in enumerate(cases,1):
            r,p,travel,initial,_ = case
            e = edges[(edges["trial"]==trial)&(edges["seconds"]<2.75)]
            widths = np.hypot(e["u1"]-e["u0"],e["v1"]-e["v0"])
            width0 = np.median(widths[e["seconds"]<.5])
            optical = 10379.5644 * (1/width0 - 1/widths)
            for stamp, movement in zip(e["seconds"],optical):
                if stamp <= 1:
                    continue
                t = stamp + 1.5 - .232
                # Compare source-time displacement with later range receipt.
                reported = initial - np.interp(t+lag,r["seconds"],r["real_mm"])
                writer.writerow([trial,stamp,np.interp(t,p["sec"],travel),movement,reported])

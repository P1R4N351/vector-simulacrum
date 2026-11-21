"""Test fixed roll without changing cube pose, material, or the measured wrist path."""
from concurrent.futures import ProcessPoolExecutor
import csv
import matplotlib.pyplot as plt
import numpy as np
from evaluate import here, run


def case(job):
    trial, angle = job
    edge = []
    def observe(model,data,t,lift):
        if 4.5 <= t <= 6.5:
            edge.append((t, -23*data.body("cube").xmat.reshape(3,3)[2,1]))
    result, _ = run((trial,"stl","overhead","weld",0,0,0), observe, wrist_roll_deg=angle)
    t, right = np.asarray(edge).T
    return (trial, angle, *np.interp([5.498107,5.997607],t,right), max(right),
            result[-3],result[-1])


if __name__ == "__main__":
    angles = [-4,-2,-1.5,-1,0,2,4]
    with ProcessPoolExecutor(4) as pool:
        results = list(pool.map(case, [(i,a) for i in range(1,6) for a in angles]))
    with (here / "wrist-roll.csv").open("w") as f:
        out = csv.writer(f,lineterminator="\n")
        out.writerow(["trial","wrist_roll_deg","right_edge_up_5_50_mm","right_edge_up_6_00_mm",
                      "peak_right_edge_up_mm","endpoint_error_mm","peak_cube_tilt_deg"])
        out.writerows(results)
    data = np.array(results)
    fig,axes = plt.subplots(1,2,figsize=(10,4))
    first = data[data[:,0]==1]
    for col, color, bounds, label in [(2,"#306b91",(7.69,13.13),"5.50 s"),(3,"#c26332",(6.70,12.40),"6.00 s")]:
        axes[0].plot(first[:,1],first[:,col],"o-",color=color,label=label)
        axes[0].axhspan(*bounds,color=color,alpha=.13)
    axes[0].set(ylabel="Right edge above left (mm)",title="Trial 1 · bands span camera estimates")
    for trial in range(2,6):
        r=data[data[:,0]==trial]
        axes[1].plot(r[:,1],r[:,5],"o-",label=f"Trial {trial}")
    axes[1].set(ylabel="Settled position error (mm)",title="Other shelf pushes")
    for ax in axes:
        ax.set(xlabel="Fixed wrist roll (degrees)",xticks=angles)
        ax.legend()
    fig.tight_layout()
    fig.savefig(here / "wrist-roll.png",dpi=160)

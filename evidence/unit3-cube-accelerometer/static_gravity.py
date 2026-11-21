"""Fit one-parameter gravity response models on upright rest and freeze for carry."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from decode import decode

here = Path(__file__).resolve().parent
actions = list(csv.DictReader((here / "action-times.csv").open()))
zero = float(next(e["pod_unix_s"] for e in actions if e["action"] == "pickup"))
disconnect = float(next(e["pod_unix_s"] for e in actions if e["action"] == "disconnect"))-zero


def packets(name):
    data,_ = decode(here/(name+".tsv"))
    time = np.array([p[0]*1e-9-zero for p in data])
    vectors = np.stack([np.frombuffer(p[3],"<i2").reshape(3,3).mean(axis=0) for p in data])/8192
    return time,vectors


def main():
    _,rest = packets("resting")
    _,undock = packets("undock-check")
    times,all_pickup = packets("pickup")
    pre = all_pickup[(times >= -1) & (times < 0)]
    high = all_pickup[(times >= 6.5) & (times < 8.3)]
    post = all_pickup[(times >= disconnect) & (times < disconnect+.9)]
    # True gravity norm is 1 g; each constrained model has just one unknown.
    m = rest.mean(axis=0)
    true_z = np.sqrt(1-m[:2]@m[:2])
    models = {
        "Nominal": (np.ones(3), np.zeros(3)),
        "Common gain": (np.full(3,np.linalg.norm(m)), np.zeros(3)),
        "Z offset": (np.ones(3), np.array([0,0,m[2]-true_z])),
        "Z gain": (np.array([1,1,m[2]/true_z]), np.zeros(3)),
    }
    windows = {"Fit (shelf)":rest,"Undock check":undock,"Before pickup":pre,
               "Tilted carry":high,"After place":post}
    fig,ax=plt.subplots(figsize=(8,4.4),layout="constrained")
    with (here/"static-gravity.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow(["capture","model","packets","gain_z","offset_z_g","mean_vector_norm_error_mg","first_half_mg","second_half_mg"])
        for i,(label,(scale,bias)) in enumerate(models.items()):
            errors=[]
            for name,data in windows.items():
                values=(data-bias)/scale
                def error(vals):
                    return 1000*(np.linalg.norm(vals.mean(axis=0))-1)
                err=error(values)
                halves=[error(chunk) for chunk in np.array_split(values,2)]
                writer.writerow([name,label,len(values),round(scale[2],7),round(bias[2],7),*np.round([err,*halves],3)])
                errors.append(err)
            ax.plot(np.arange(len(errors))+.04*(i-1.5),errors,"o",label=label)
    ax.axhline(0,color="gray",ls=":")
    ax.set(xticks=range(len(windows)),xticklabels=list(windows),ylabel="Recovered gravity magnitude minus 1 g (mg)",title="Upright calibration checked during the tilted carry")
    ax.legend(ncol=2)
    fig.savefig(here/"static-gravity.png",dpi=155)


if __name__=="__main__":
    main()

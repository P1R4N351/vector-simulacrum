from pathlib import Path
import csv
import matplotlib.pyplot as plt
import numpy as np

files=["unit-A","unit-B","unit-C","before","after"]
fig,axes=plt.subplots(2,1,figsize=(8,5.5),layout="constrained")
with open("summary.csv","w",newline="") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["source","reports","gyro_changes","accel_changes","disagree","change_fraction","effective_interval_ms"])
    for i,name in enumerate(files):
        a=np.genfromtxt(name+".csv",names=True,delimiter=",")
        if name.startswith("unit"):
            assert np.all(a["status"].astype(int)&4096) and not np.any(a["wl"]) and not np.any(a["wr"])
        def changes(keys):
            return np.any(np.diff(np.column_stack([a[k] for k in keys]),axis=0)!=0,axis=1)
        gyro=changes(("gx","gy","gz")); accel=changes(("ax","ay","az"))
        hits=a["s"][1:][gyro]; t=a["s"]-a["s"][0]
        w.writerow((name,len(a),sum(gyro),sum(accel),sum(gyro!=accel),gyro.mean(),1000*np.mean(np.diff(hits))))
        axes[0].eventplot(t[1:][gyro & (t[1:]<=3)],lineoffsets=i,linelengths=.6)
        axes[1].plot(t,np.r_[0,np.cumsum(gyro)],label=name,ls="--" if name=="after" else "-")
axes[0].set(yticks=range(5),yticklabels=files,xlabel="Seconds",title="Fresh gyro triples in the first three seconds")
axes[1].set(xlabel="Seconds",ylabel="Cumulative changed reports");axes[1].legend()
fig.savefig("cadence.png",dpi=180)

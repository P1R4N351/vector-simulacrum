"""Evaluate the stock conversion on fresh reports, including untouched unit C."""
import csv
import numpy as np
import matplotlib.pyplot as plt
step=2*9810/32767
fig,(a,b)=plt.subplots(1,2,figsize=(9,4),layout="constrained")
with open("counts-scores.csv","w",newline="") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["source","axis","max_error_count","fresh_sd_mg","lag_one_correlation"])
    for i,(name,file) in enumerate([("A","unit-A"),("B","unit-B"),("C","unit-C"),("Before","after"),("After","counts-replay")]):
        r=np.genfromtxt(file+".csv",names=True,delimiter=",")
        raw=np.column_stack([r[k] for k in ("ax","ay","az")])
        raw=raw[np.r_[True,np.any(np.diff(raw,axis=0)!=0,axis=1)]]
        counts=raw/step
        err=abs(counts-counts.round())
        # Drop 12 fresh samples for the dispersion analysis, including settling.
        steady=raw[12:]/9.80665
        sd=steady.std(axis=0,ddof=1)
        for j,axis in enumerate("XYZ"):
            w.writerow([name,axis,err[:,j].max(),sd[j],np.corrcoef(steady[:-1,j],steady[1:,j])[0,1]])
        a.scatter(np.arange(3)+(i-2)*.09,np.maximum(err.max(axis=0),1e-12),label=name)
        b.scatter(np.arange(3)+(i-2)*.09,sd,label=name)
for ax in (a,b):ax.set(xticks=range(3),xticklabels=list("XYZ"))
a.set(yscale="log",ylabel="Maximum distance to integer (counts)",title="HAL count spacing")
b.set(ylabel="Fresh-reading standard deviation (mg)",title="Stationary dispersion")
b.legend()
fig.savefig("counts.png",dpi=165)

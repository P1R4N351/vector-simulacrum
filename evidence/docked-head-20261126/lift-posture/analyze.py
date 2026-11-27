from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt

def read(path):
    lines=Path(path).read_text().splitlines()
    lines=[line + ","*(15-line.count(",")) for line in lines]
    return np.genfromtxt(lines,names=True,delimiter=",",dtype=None,encoding="utf8")
def crossing(time,x,threshold):
    j=np.flatnonzero((x[:-1]<threshold)&(x[1:]>=threshold))
    if not len(j): return np.nan
    i=j[0]
    return time[i]+(time[i+1]-time[i])*(threshold-x[i])/(x[i+1]-x[i])

k=read("active.csv"); s=k[k["event"]=="state"]
commands=k[k["event"]=="head"]
fig,ax=plt.subplots(1,3,figsize=(11,3.8),layout="constrained")
with open("moves.csv","w",newline="") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["block","direction","lift_mm","transit_s","peak_abs_gyro_y_rad_s","sample_count"])
    for i,c in enumerate(commands):
        if i%3==0:continue
        target=float(c["target"]); sign=1 if target>0 else -1
        seg=s[(s["s"]>=c["s"])&(s["s"]<c["s"]+1.25)]
        t=seg["s"]-c["s"]; angle=seg["head_deg"]*sign
        t0=crossing(t,angle,-5 if sign==1 else -18)
        t1=crossing(t,angle,18 if sign==1 else 5)
        w.writerow((i//3+1,"raise" if sign==1 else "lower",np.median(seg["lift_mm"]),t1-t0,max(abs(seg["gy"])),len(t)))
        a=ax[0 if sign==1 else 1]
        a.plot(t-t0,seg["head_deg"],".-",ms=3,color="#176c9e" if float(c["condition"])==0 else "#c25720",alpha=.8)
        a.set(title="Head raise" if sign==1 else "Head lower",xlabel="Seconds from first angle crossing",ylabel="Head encoder (degrees)",xlim=(-.22,.55))
for label,color in [("Lift low","#176c9e"),("Lift raised","#c25720")]:
    ax[0].plot([],[],color=color,label=label)
ax[0].legend()
b=read("passive.csv"); b=b[b["event"]=="state"]
for key in ("gx","gy","gz"):ax[2].plot(b["s"],np.rad2deg(b[key]),lw=.8,label=key)
ax[2].set(title="Other unit · passive on dock",xlabel="Seconds",ylabel="Gyro (°/s)");ax[2].legend()
fig.savefig("docked-pair.png",dpi=165)
for name,a in [("active",s),("passive",b)]:
    print(name,len(a),np.ptp(a["s"]),np.ptp(a["head_deg"]),np.ptp(a["lift_mm"]),np.max(abs(a["wl"])),np.max(abs(a["wr"])),len(np.unique(a["image_stamp"])),np.median(np.diff(a["s"])))

"""Compare simulated and captured transits, including the SDK sample times."""
import csv
import matplotlib.pyplot as plt
import numpy as np

with open("active.csv") as f:
    rows=list(csv.DictReader(f))
states=[r for r in rows if r["event"]=="state"]
commands=[r for r in rows if r["event"]=="head"]
sim=np.genfromtxt("posture-replay.csv",names=True,delimiter=",")
assert np.all(sim["dock"]==1)
def crossing(t,x,bound):
    i=np.flatnonzero((x[:-1]<bound)&(x[1:]>=bound))
    if not i.size:raise ValueError("Missing crossing")
    j=i[0]
    return t[j]+(bound-x[j])*(t[j+1]-t[j])/(x[j+1]-x[j])
def metrics(t,angle,gyro,sign):
    a,b=(-5,18) if sign==1 else (-18,5)
    p=crossing(t,angle*sign,a)
    q=crossing(t,angle*sign,b)
    return p,q-p,np.max(abs(gyro))
fig,axes=plt.subplots(2,2,figsize=(9,6),sharex=True,layout="constrained")
with open("posture-scores.csv","w",newline="") as f:
    out=csv.writer(f,lineterminator="\n")
    out.writerow(["block","direction","source","target_lift_mm","first_cross_after_cue_s","transit_s","abs_peak_gyro_rad_s","observed_lift_mm"])
    for i,c in enumerate(commands):
        if i%3==0:continue
        cue=float(c["s"]);sign=np.sign(float(c["target"]))
        hw=[r for r in states if cue<=float(r["s"])<cue+1.25]
        ht,ha,hg,hl=([float(r[k]) for r in hw] for k in ("s","head_deg","gy","lift_mm"))
        t=np.asarray(ht)
        a=np.asarray(ha);g=np.asarray(hg)
        mask=(sim["s"]>=cue)&(sim["s"]<cue+1.25)
        cases=[("physical",t,a,g,hl),("sim_5ms",sim["s"][mask],sim["head_deg"][mask],sim["gy"][mask],sim["lift_mm"][mask]),
               ("sim_sdk",t,*[np.interp(t,sim["s"],sim[k]) for k in ("head_deg","gy","lift_mm")])]
        indices=np.searchsorted(sim["s"],t,side="right")-1
        cases.append(("sim_published",t,*[sim[k][indices] for k in ("pub_head","pub_gy","pub_lift")]))
        for name,time,angle,gyro,lift in cases:
            p,transit,peak=metrics(time,angle,gyro,sign)
            out.writerow((i//3+1,"raise" if sign==1 else "lower",name,np.median(hl),p-cue,transit,peak,np.median(lift)))
            if name not in ("physical","sim_5ms"):continue
            col=0 if sign==1 else 1
            color="#117eaa" if name=="physical" else "#bd551f"
            for j,y in enumerate((angle,gyro)):
                axes[j,col].plot(time-p,y,".-" if name=="physical" else "-",ms=2,color=color,alpha=.65,lw=1,
                                 label=name if i in (1,2) else None)
for col,title in enumerate(("Raises","Lowers")):
    axes[0,col].set(title=title,ylabel="Head encoder (°)")
    axes[1,col].set(xlabel="Seconds from first angle crossing",ylabel="Head gyro Y (rad/s)",xlim=(-.2,.55))
axes[0,0].legend()
fig.savefig("posture-overlay.png",dpi=170)

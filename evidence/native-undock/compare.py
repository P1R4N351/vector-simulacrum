"""Select a single clip and dispatch offset on A–C and freeze both for D–F."""
import csv,gzip
import numpy as np
import matplotlib.pyplot as plt

def rows(file):
    with gzip.open(file,"rt") as f:
        return list(csv.DictReader(f))

obs={}
for name in "ABCDEF":
    physical=rows(f"../unit3-physical-repeats/{name}/robot.csv.gz")
    with open(f"../unit3-physical-repeats/{name}/actions.csv") as f:
        end=next(float(r["receipt_s"]) for r in csv.DictReader(f) if r["event"]=="end" and r["action"]=="undock")
    active=[r for r in physical if 0<=float(r["receipt_s"])<=end]
    t=np.array([float(r["receipt_s"]) for r in active])
    wheels=np.array([[float(r[k]) for k in ["left_mmps","right_mmps"]] for r in active])
    off=next(float(r["receipt_s"]) for r in active if not int(r["status"])&4096)
    obs[name]=(t,wheels,off)
pred=np.genfromtxt("predictions.csv",names=True,dtype=None,encoding="utf8",delimiter=",")
clips={k:pred[pred["clip"]==k] for k in dict.fromkeys(pred["clip"])}
def metrics(clip,lag,trial):
    t,meas,off=obs[trial]
    sim=clips[clip]
    est=np.array([np.interp(t-lag,sim["t"],sim[k]) for k in ["left","right"]]).T
    e=est-meas
    first=sim["t"][np.flatnonzero(sim["on"]==0)[0]]+lag if np.any(sim["on"]==0) else np.nan
    return np.mean(e**2)**.5,np.mean(e[np.abs(t-off)<=1]**2)**.5,first-off
def loss(choice):
    return np.mean([metrics(*choice,n)[0] for n in "ABC"])
lags=np.arange(0,1.01,.05)
fit={c:min([(c,l) for l in lags],key=loss) for c in clips}
best=min((v for k,v in fit.items() if k!="canonical"),key=loss)
if __name__ == "__main__":
    fig,panels=plt.subplots(2,3,figsize=(12,6),layout="constrained")
    with open("scores.csv","w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["trial","clip","dispatch_s","wheel_rmse_mmps","release_window_rmse_mmps","release_error_s"])
        for n,ax in zip("ABCDEF",panels.flat):
            for choice in fit.values():
                w.writerow([n,*choice,*metrics(*choice,n)])
            t,meas,edge=obs[n]
            ax.plot(t,meas.mean(axis=1),color="black",lw=1,label="Physical")
            for c,lag in [fit["canonical"],best]:
                s=clips[c]; ax.plot(s["t"]+lag,(s["left"]+s["right"])/2,lw=1,label=c)
            ax.axvline(edge,color="grey",ls=":",label="Physical first off")
            ax.set(xlim=(0,t[-1]),ylim=(-160,170),title=f"Session {n}",xlabel="Seconds from SDK request",ylabel="Mean wheel speed (mm/s)")
    panels.flat[0].legend(fontsize=7)
    fig.savefig("native.png",dpi=150)
    print("Chosen",best,"default",fit["canonical"])
    for k,v in fit.items():print(k,v[1],*[round(np.mean([metrics(*v,n)[i] for n in group]),2) for group in ["ABC","DEF"] for i in [0,1,2]])

"""Score release angular velocities using the observed low-force packet slots."""
import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
runs = [("shelf","../S22B/cube.csv.gz",37.020336,1),
        ("pickup","../../R5/cube.csv.gz",39.980848,0)]
scores = {}
waves = {}
refs = {}
for name, source, timestamp, slot in runs:
    with gzip.open(here/source,"rt") as stream:
        rows = list(csv.DictReader(stream))
    idx = next(i for i,r in enumerate(rows) if float(r["sec"])==timestamp and int(r["slot"])==slot)
    ref = np.array([[int(r[k]) for k in "xyz"] for r in rows[idx:idx+5]])/8192
    refs[name] = ref
    a = np.vstack([np.loadtxt(here/(name+suffix),delimiter=",",skiprows=1,dtype=int)
                   for suffix in (".csv.gz","-mid.csv.gz")])
    for wx,wy,phase in np.unique(a[:,[10,11,2]],axis=0):
        block = a[(a[:,10]==wx)&(a[:,11]==wy)&(a[:,2]==phase)]
        assert np.all(block[:,6]==np.arange(len(block))%3)
        xyz = block[:,7:10]/8192
        key = (name,wx,wy,phase)
        options = [xyz[i:i+5] for i in range(slot,len(xyz)-4,3)
                   if np.linalg.norm(xyz[i:i+2],axis=1).max()<.05]
        assert len(options)<=1, "ambiguous impact window"
        if options:
            waves[key] = options[0]
            scores[key] = np.sqrt(np.mean((options[0][2:]-ref[2:])**2))
        else:
            scores[key] = np.nan
with (here/"scores.csv").open("w") as stream:
    w=csv.writer(stream,lineterminator="\n")
    w.writerow(("run","omega_x_radps","omega_y_radps","phase_ms","impact_rmse_g"))
    for k,v in scores.items():
        w.writerow((*k,v if np.isfinite(v) else ""))
def best(name):
    return min((k for k,v in scores.items() if k[0]==name and np.isfinite(v)),key=scores.get)
shelf = best("shelf")
print("shelf selected",shelf,scores[shelf])
for run in refs:
    for rate in [(0,0),shelf[1:3]]:
        items=[(k[-1],round(v,4)) for k,v in scores.items() if k[:3]==(run,*rate) and np.isfinite(v)]
        print(run,rate,items)
    print(run,"own best",best(run),scores[best(run)])
fig,axes=plt.subplots(1,2,figsize=(11,5),layout="constrained",sharey=True)
labels=["Rest"]+[f"X {r:+}" for r in (-4,-3,-2,2,4)]+[f"Y {r:+}" for r in (-4,-2,2,4)]
rates=[(0,0)]+[(r,0) for r in (-4,-3,-2,2,4)]+[(0,r) for r in (-4,-2,2,4)]
for ax,run in zip(axes,refs):
    data=np.array([[scores[(run,*rate,p)] for p in range(0,30,5)] for rate in rates])
    im=ax.imshow(data,aspect="auto",vmin=0,vmax=2.5,cmap="viridis")
    ax.set(xticks=range(6),xticklabels=range(0,30,5),yticks=range(len(rates)),yticklabels=labels,
           xlabel="Release phase (ms)",title=run.capitalize())
    for (i,j),v in np.ndenumerate(data):
        if np.isfinite(v):
            ax.text(j,i,f"{v:.2f}",ha="center",va="center",fontsize=8,color="white" if v<1.25 else "black")
axes[0].set_ylabel("Initial cube-axis angular velocity (rad/s)")
fig.colorbar(im,ax=axes,label="Impact RMS error (g)")
fig.suptitle("Cube release rotation · blank cells miss the observed low-force pair")
fig.savefig(here/"rotation.png",dpi=155)

fig,axes=plt.subplots(2,3,figsize=(10,5.5),sharex=True,sharey=True,layout="constrained")
for axrow,run in zip(axes,refs):
    valid=[k for k in waves if k[0]==run]
    still=min((k for k in valid if k[1:3]==(0,0)),key=scores.get)
    rotating=min((k for k in valid if k[1:3]==(-3,0)),key=scores.get)
    for j,ax in enumerate(axrow):
        ax.plot(refs[run][:,j],"ko-",label="Recorded")
        ax.plot(waves[still][:,j],"C0.--",label="Zero initial rotation")
        ax.plot(waves[rotating][:,j],"C3.:",label="Camera-selected −3 rad/s")
        ax.set_title(f"{run.capitalize()} · {'XYZ'[j]}")
        ax.set_xticks(range(5),labels=range(1,6))
    axrow[0].set_ylabel("Acceleration (g)")
axes[0,0].legend(fontsize=7)
for ax in axes[-1]: ax.set_xlabel("Sample slot")
fig.savefig(here/"slot-overlay.png",dpi=155)

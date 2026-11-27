"""Quantify idle shaft motion and check a held-out physical departure."""
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
here=Path(__file__).resolve().parent
def read(p):
    return np.genfromtxt(p,delimiter=",",names=True)
fig,axes=plt.subplots(1,3,figsize=(12,4),layout="constrained")
for mode in ("base","hinges","dock","both","priority"):
    r=read(here/f"{mode}-screen.csv")
    dx=r["x_mm"]-r["x_mm"][0]
    shaft=(r["left_mm"]+r["right_mm"])/2
    shaft-=shaft[0]
    print(mode,round(dx[-1],3),round(shaft[-1],3))
    axes[0].plot(r["s"],dx,label=mode)
    axes[1].plot(r["s"],shaft,label=mode)
rails=read(here/"long-both.csv")
before=rails[rails["s"]<=70.7]
print("v1 both drift",before["x_mm"][-1]-before["x_mm"][0],"contact",min(before["contact"]),
      "first sampled off",rails["s"][rails["contact"]==0][0])
cam=read(here.parent/"unit3-w34-return/departure-hubs.csv")
t=cam["t_s"]
scale=48/(cam["rear_x"][0]-cam["front_x"][0])
for field in ("rear_x","front_x"):
    x=(cam[field][0]-cam[field]+cam["static_x"]-cam["static_x"][0])*scale
    axes[2].plot(t,x,".-",label=field.split("_")[0]+" hub")
for mode in ("base","both"):
    r=read(here/f"depart-{mode}.csv")
    x=r["x_mm"]-np.interp(t[0],r["s"],r["x_mm"])
    idx=np.flatnonzero(r["charger"]==0)
    print("v2",mode,"first off",r["s"][idx[0]] if len(idx) else None,
          "travel at physical off",np.interp(0,r["s"],x),"at sim off",x[idx[0]] if len(idx) else None)
    axes[2].plot(r["s"],x,label=mode)
    if len(idx): axes[2].scatter(r["s"][idx[0]],x[idx[0]],marker="x")
axes[0].set(ylabel="Downhill chassis travel (mm)",xlabel="Idle seconds")
axes[1].set(ylabel="Mean drive-shaft travel (mm)",xlabel="Idle seconds")
axes[2].set(xlim=(-.48,.55),ylim=(-1,20),ylabel="Travel from first camera frame (mm)",xlabel="Seconds from physical first off")
axes[2].axvspan(-.081,0,color="gray",alpha=.13)
for ax in axes:
    ax.grid(alpha=.2)
axes[0].legend(fontsize=8)
axes[2].legend(fontsize=8)
fig.savefig(here/"support.png",dpi=160)

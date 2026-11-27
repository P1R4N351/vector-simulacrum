"""Compare a lift-only sequence and an idle replay with identical dock initialization."""
from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
fig,axes=plt.subplots(2,1,sharex=True,layout="constrained",figsize=(9,5))
with (here/"recording.csv").open() as f:
    states=[r for r in csv.DictReader(f) if r["event"]=="state"]
axes[1].step([float(r["s"]) for r in states],[int(r["on_dock"]) for r in states],where="post",label="Physical contact",color="k",ls=":")
for mode in ("sequence","idle"):
    with (here/f"dock-{mode}.csv").open() as f:
        rows=list(csv.DictReader(f))
    t=np.array([float(r["s"]) for r in rows])
    xy=np.array([[float(r[k]) for k in ("x_mm","y_mm")] for r in rows])
    dock=np.array([int(r["on_dock"]) for r in rows])
    if mode=="sequence":
        with (here/"replay.csv").open() as f:
            replay=list(csv.DictReader(f))
        assert np.array_equal(dock,[int(r["on_dock"]) for r in replay])
    delta=xy-xy[0]
    print(mode,"first loss",t[dock==0][:1],"off count",sum(dock==0),"final delta xy",delta[-1],
          "platform off",sum(r["on_platform"]=="0" for r in rows))
    axes[0].plot(t,np.linalg.norm(delta,axis=1),label=f"Sim {mode}")
    axes[1].step(t,dock,where="post",label=f"Sim {mode}",alpha=.8)
axes[0].set_ylabel("Sim chassis XY travel\nfrom first pose (mm)")
axes[1].set(yticks=[0,1],yticklabels=["Off","On"],xlabel="Seconds",ylabel="Charger contact",ylim=(-.1,1.1))
for ax in axes:
    ax.legend()
    ax.grid(alpha=.2)
fig.savefig(here/"dock.png",dpi=160)

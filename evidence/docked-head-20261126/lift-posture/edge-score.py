"""Compare angle-driven WireOS head-speed estimates with the physical Y gyro."""
import csv
import numpy as np
import matplotlib.pyplot as plt

with open("active.csv") as f:
    rows=list(csv.DictReader(f))
commands=[r for r in rows if r["event"]=="head"]
# Each block first positions the head, then raises and lowers it.
moves=[(int(i/3)+1,r) for i,r in enumerate(commands) if i%3 in (1,2)]
states=[r for r in rows if r["event"]=="state"]
t=np.array([float(r["s"]) for r in states])
gyro=-np.degrees([float(r["gy"]) for r in states])
replay=np.genfromtxt("edge-replay.csv",names=True,delimiter=",")
fig,axes=plt.subplots(2,2,figsize=(10,6),sharex=True,layout="constrained")
with open("edge-scores.csv","w") as f:
    writer=csv.writer(f,lineterminator="\n")
    writer.writerow(["block","direction","samples","frame_clock_rmse_dps","edge_clock_rmse_dps"])
    for block,cmd in moves:
        start=float(cmd["s"])
        use=(t>=start)&(t<start+1)
        ax=axes[0 if float(cmd["target"])>0 else 1,(block-1)%2]
        held=block>=3
        if held: ax.plot(t[use]-start,gyro[use],"ko-",ms=3,label="Physical gyro")
        scores=[]
        for field,label in [("frame_clock_dps","Frame clock"),("edge_clock_dps","Edge clock")]:
            y=np.interp(t[use],replay["s"],replay[field])
            scores.append(np.mean((gyro[use]-y)**2)**.5)
            if held: ax.plot(t[use]-start,y,label=label)
        direction="raise" if float(cmd["target"])>0 else "lower"
        writer.writerow([block,direction,sum(use),*scores])
        if held: ax.set(title=f"Block {block}, {direction}",ylabel="Head speed (°/s)",xlabel="Seconds after command")
axes[0,0].legend(fontsize=8)
fig.savefig("edge-gyro.png",dpi=150)

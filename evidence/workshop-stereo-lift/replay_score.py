import csv
import numpy as np
import matplotlib.pyplot as plt

def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))

rows=read("sdk.csv")
cmd=[r for r in rows if r["event"]=="command"]
done=[r for r in rows if r["event"]=="done"]
state=[r for r in rows if r["event"]=="state"]
t=np.array([float(r["relative_s"]) for r in state])
h=np.array([float(r["lift_mm"]) for r in state])
s=np.genfromtxt("replay.csv",names=True,delimiter=",")
eye=np.genfromtxt("eye-estimates.csv",names=True,delimiter=",")
finish=read("returns.csv")
assert np.allclose(s["s"],t) and len(cmd)==len(done)==len(finish)==8
starts=np.array([float(r["relative_s"]) for r in cmd])
moving=np.abs(np.gradient(h,t))>5
grid=np.arange(-.2,.201,.005)
train=(t>=0)&(t<starts[4])&moving
def predicted(delay):
    return np.interp(t-delay,t,s["lift_mm"])
delay=grid[np.argmin([np.mean(abs(predicted(d)[train]-h[train])) for d in grid])]
held=(t>=starts[4])&moving
for d in [0,delay]:
    print("delay, held moving MAE",round(d,3),np.mean(abs(predicted(d)[held]-h[held])))
fig,axes=plt.subplots(2,1,figsize=(10,6),layout="constrained",sharex=True)
axes[0].plot(t,h,label="Encoder",color="black")
axes[0].plot(t,s["lift_mm"],label="Canonical sim",color="tab:orange")
e=eye[eye["held"]>0]
for key in ["left_mm","right_mm"]:
    axes[0].scatter(e["t"],e[key],s=5,label=key.split("_")[0]+" camera")
axes[1].plot(t,s["lift_mm"]-h,label="Canonical")
axes[1].plot(t,predicted(delay)-h,label=f"Shift from first four ({delay*1000:.0f} ms)")
for ax in axes:
    ax.axvline(starts[4],ls="--",color="gray")
    ax.legend(ncol=2)
axes[0].set(ylabel="Lift height (mm)",title="Eight docked lift moves · reserved repeat after dashed line")
axes[1].set(ylabel="Sim minus encoder (mm)",xlabel="Seconds since first command")
fig.savefig("canonical-overlay.png",dpi=150)
with open("move-errors.csv","w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["move","target_mm","physical_return_s","sim_return_s","moving_packets","canonical_moving_mae_mm","shifted_moving_mae_mm","settled_encoder_mm","settled_sim_mm"])
    for i,start in enumerate(starts):
        segment=(t>=start)&(t<(starts[i+1] if i<7 else t[-1]+.01))
        m=segment&moving
        rest=segment&(t>float(done[i]["relative_s"])+.15)
        w.writerow([i+1,cmd[i]["target"],float(done[i]["relative_s"])-start,finish[i]["sim_elapsed_s"],sum(m),np.mean(abs(s["lift_mm"][m]-h[m])),np.mean(abs(predicted(delay)[m]-h[m])),np.median(h[rest]),np.median(s["lift_mm"][rest])])
actual=np.array([float(r["relative_s"]) for r in done])-starts
sim_return=np.array([float(r["sim_elapsed_s"]) for r in finish])
extra=np.median((actual-sim_return)[:4])
print("Return offset",extra,"held MAE before/after",*[np.mean(abs((actual-sim_return)[4:]-d)) for d in (0,extra)])
print("held eye MAE vs sim",*[np.mean(abs(np.interp(e["t"],t,s["lift_mm"])-e[k])) for k in ("left_mm","right_mm")])

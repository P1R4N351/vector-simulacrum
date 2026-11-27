"""Score central lift travel on the same SDK receipt clock for both traces."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).parent
def read(name):
    with (here/name).open() as f:
        return list(csv.DictReader(f))
rows = read("recording.csv")
states = [r for r in rows if r["event"]=="state"]
model = read("replay.csv")
t = np.array([float(r["s"]) for r in states])
real = np.array([float(r["lift_mm"]) for r in states])
sim = np.array([float(r["lift_mm"]) for r in model])
assert np.array_equal(t,[float(r["s"]) for r in model])
assert all(int(r["on_dock"]) for r in states+model)
commands = [r for r in rows if r["event"]=="command"]
assert len(commands)==16
def cross(times, vals, level):
    indices = np.flatnonzero((vals[:-1]>level)&(vals[1:]<=level))
    if not len(indices):
        return np.full(3,np.nan)
    n = indices[0]
    estimate = np.interp(level,vals[n:n+2][::-1],times[n:n+2][::-1])
    return np.array([times[n],estimate,times[n+1]])

fig,axes=plt.subplots(2,2,figsize=(9,6.3),layout="constrained",sharex=True,sharey="row")
with (here/"scores.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("trial from_mm duration_s trace start_mm final_mm crossing20_ms central60_ms low_ms high_ms".split())
    for j in range(1,len(commands),2):
        height=float(commands[j-1]["target_mm"])
        dur=float(commands[j]["duration_s"])
        cue=float(commands[j]["s"])
        end=float(commands[j+1]["s"]) if j+1<len(commands) else t[-1]+.01
        sel=np.arange(np.searchsorted(t,cue)-1,np.searchsorted(t,end))
        times=t[sel]-cue
        ax=axes[int(height>50),int(dur>.2)]
        for label,signal,color in [("Physical",real,"C0"),("Canonical",sim,"C1")]:
            vals=signal[sel]
            ax.plot(times,vals,".-" if label=="Physical" else ".--",lw=1,ms=3,color=color,label=label)
            a=cross(times,vals,height-.2*(height-32))
            b=cross(times,vals,height-.8*(height-32))
            w.writerow([j//2+1,height,dur,label,vals[0],vals[-1],1000*a[1],1000*(b[1]-a[1]),
                        1000*np.maximum(0,b[0]-a[2]),1000*(b[2]-a[0])])
        ax.set_title(f"{height:.0f} to 32 mm · {dur*1000:.0f} ms requested")
        ax.grid(alpha=.2);ax.set_xlim(-.08,1.1)
for ax in axes[-1]:ax.set_xlabel("Seconds after SDK command")
for ax in axes[:,0]:ax.set_ylabel("Lift height (mm)")
handles,labels=axes[0,0].get_legend_handles_labels()
axes[0,0].legend(handles[:2],labels[:2])
fig.savefig(here/"comparison.png",dpi=165)

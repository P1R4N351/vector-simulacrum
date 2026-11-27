"""Score marked passive and override intervals on their shared host clock."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).parent
fig, axes = plt.subplots(4, 2, figsize=(11, 8), sharex="col", layout="constrained")
scores = []
for label in ["pilot", "session-1", "session-2"]:
    with (root / (label+".csv")).open() as f:
        rows = list(csv.DictReader(f))
    events = {r["event"]:float(r["receipt_s"]) for r in rows if r["event"]!="state"}
    start, request, grant = [events[k] for k in ("subscribed","request","granted")]
    states = [r for r in rows if r["event"] == "state"]
    t,angle,gyro,status = np.array([[float(r[k]) for k in ("receipt_s","head_deg","gyro_y","status")] for r in states]).T
    end = grant+8
    assert np.all(status[t<end].astype(int) & 4096)
    anim = (status.astype(int) & 64)!=0
    calm = (status.astype(int) & 1024)!=0
    passive = (t >= start) & (t < request)
    hold = (t >= grant) & (t < end)
    active = t[hold & anim]-grant
    speeds = np.abs(gyro[hold])
    onset = np.flatnonzero((speeds[:-1]>.1) & (speeds[1:]>.1))
    scores.append([label, round(request-start,3), round((grant-request)*1000), int(sum(passive)),sum(calm[passive]),sum(anim[passive]),round(np.ptp(angle[passive]),2),
                   int(sum(hold)),sum(calm[hold]),sum(anim[hold]),round(active[0],3) if len(active) else "",round(min(t[hold & ~calm])-grant,3) if np.any(hold & ~calm) else "",round(np.ptp(angle[hold]),2),
                   round((t[hold]-grant)[onset[0]],3) if len(onset) else "",round(active[-1],3) if len(active) else ""])
    for j, (origin,mask) in enumerate([(start,passive | hold),(grant,hold)]):
        for ax,values in zip(axes[:,j],[angle,gyro,anim.astype(int),calm.astype(int)]):
            ax.plot(t[mask]-origin, values[mask], lw=1.3, alpha=.85,label=label,drawstyle="steps-post" if ax in (axes[2,j],axes[3,j]) else "default")
axes[0,0].set_title("Aligned to passive subscription")
axes[0,1].set_title("Aligned to confirmed override")
for ax,label in zip(axes[:,0],["Head encoder (°)","Gyro Y (rad/s)","SDK animating bit","Calm power mode"]):
    ax.set_ylabel(label)
axes[0,1].legend()
for ax in axes[3]:ax.set_xlabel("Time (s)")
fig.savefig(root/"deferred-grant.png",dpi=170)
with (root/"summary.csv").open("w") as f:
    writer=csv.writer(f)
    writer.writerow("session passive_s grant_wait_ms passive_packets passive_calm passive_animating passive_head_range_deg grant_packets grant_calm grant_animating first_animating_s first_calm_off_s grant_head_range_deg first_gyro_motion_s last_animating_s".split())
    writer.writerows(scores)

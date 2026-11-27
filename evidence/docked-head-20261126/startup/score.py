"""Align encoder, gyro and SDK animation bit to stream subscription and raise response."""
from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt

base = Path(__file__).parent
paths = sorted(base.glob("session-*.csv"))
assert len(paths) == 4
fig, axes = plt.subplots(3, 2, figsize=(12, 7), sharex="col", layout="constrained")
scores = []
for i, path in enumerate(paths):
    with path.open() as f:
        rows = list(csv.DictReader(f))
    def event(name, index=""):
        return float(next(r for r in rows if r["event"] == name and (r["trial"] or "") == index)["t_s"])
    subscription = event("subscribed")
    raise_return = event("return", "1")
    lower = event("command", "2")
    states = [r for r in rows if r["event"] == "state"]
    t, head, gyro, status = np.array([[float(r[k]) for k in ("t_s","head_deg","gyro_y_rad_s","status")] for r in states]).T
    flags = status.astype(int)
    assert np.all(flags & 4096)
    assert all(int(r["head_deg"]) == 0 for r in rows if r["event"] == "return")
    anim = (flags & 64) != 0
    during = (t >= raise_return) & (t < lower)
    pre = head[t <= raise_return][-1]
    baseline = (t >= subscription) & (t < event("command","0"))
    sim = np.genfromtxt(base/f"canonical-{i+1}.csv", names=True, delimiter=",")
    assert np.all(sim["charger"] == 1)
    k = np.searchsorted(sim["t_s"], t[during], side="right")-1
    predicted = sim["head_deg"][k]
    elapsed = t[during] - raise_return
    ys = head[during]
    gs = gyro[during]
    moving = np.flatnonzero((gs[:-1] > .1) & (gs[1:] > .1))
    observed = (t >= subscription) & (t < lower) & anim
    last_anim = round(max(t[observed])-subscription,3) if observed.any() else ""
    scores.append([i+1,len(t),round(event("command","0")-subscription,3),round(raise_return-subscription,3),
                   round(pre,2),round(max(0,pre-min(ys)),2),round(max(abs(gs)),3),round(elapsed[moving[0]],3) if moving.size else "",
                   round(np.sqrt(np.mean((ys-predicted)**2)),2),sum(anim[during]),sum(during),sum(anim[baseline]),sum(baseline),round(np.ptp(head[baseline]),2),last_anim])
    if i==0:
        axes[0,1].plot(elapsed, predicted, "k:", lw=2,label="Canonical hold")
    label = f'Session {i+1} ({[1,9,9,1][i]} s)'
    for col, (origin, mask) in enumerate([(subscription,t < lower),(raise_return,during)]):
        for row,signal in enumerate([head,gyro,anim.astype(float)]):
            ax = axes[row,col]
            ax.plot(t[mask]-origin, signal[mask], label=label, lw=1.4, alpha=.8,
                    drawstyle="steps-post" if row==2 else "default")
axes[0,0].set_title('Full session through end of raised hold')
axes[0,1].set_title('Raised hold, before lower command')
axes[0,0].legend(fontsize=8)
axes[0,1].legend(fontsize=8)
for row,name in enumerate(["Head angle (°)", "Gyro Y (rad/s)", "SDK animating bit"]):
    axes[row,0].set_ylabel(name)
axes[-1,0].set_xlabel('Seconds after subscribing under override')
axes[-1,1].set_xlabel('Seconds after raise returned')
fig.savefig(base/'acquisition-overlay.png',dpi=160)
with (base/'scores.csv').open('w') as f:
    w=csv.writer(f)
    w.writerow(['session','states','initial_wait_s','raise_return_since_subscription_s','angle_at_return_deg','max_hold_drop_deg','max_abs_hold_gyro_rad_s','lowering_gyro_onset_s','hold_angle_rmse_deg','hold_animated_packets','hold_packets','initial_animated_packets','initial_packets','initial_head_range_deg','last_animated_before_lower_since_subscription_s'])
    w.writerows(scores)

"""Compare uninterrupted waits between the raise response and the next command."""
from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
fig, axes = plt.subplots(2, 3, figsize=(13, 6), sharex="col", layout="constrained")
summary = []
for col, path in enumerate([root.parent / "unit2.csv", root / "holds.csv", root / "check.csv"]):
    with path.open() as f:
        rows = list(csv.DictReader(f))
    states = [r for r in rows if r["event"] == "state"]
    times, angle, speed = np.array([[float(r[k]) for k in ("t_s", "head_deg", "gyro_y_rad_s")] for r in states]).T
    commands = [r for r in rows if r["event"] == "command"]
    returns = {r["trial"]:r for r in rows if r["event"] == "return"}
    model = np.genfromtxt([root.parent / "canonical.csv", root / "canonical.csv", root / "canonical-check.csv"][col], delimiter=",", names=True)
    assert np.all(model["charger"] == 1)
    for index, (cue, following) in enumerate(zip(commands, commands[1:])):
        if float(cue["target_deg"]) <= 0:
            continue
        response = returns[cue["trial"]]
        assert int(response["head_deg"]) == 0
        start,end = float(response["t_s"]),float(following["t_s"])
        mask = (times >= start) & (times < end)
        t, a, g = times[mask] - start, angle[mask], speed[mask]
        before = angle[times <= start][-1]
        # Two successive positive Y gyro samples confirm lowering in this mounting.
        moving = np.flatnonzero((g[:-1] > .1) & (g[1:] > .1))
        dropped = np.flatnonzero(before - a >= 2)
        trial = (index+1)//2
        sim = model["head_deg"][np.searchsorted(model["t_s"], times[mask], side="right")-1]
        flags = [int(row["status"]) for row in states if start <= float(row["t_s"]) < end]
        animated = sum(bool(flag & 64) for flag in flags)
        in_position = sum(bool(flag & 512) for flag in flags)
        summary.append([path.stem,trial,animated,in_position,cue["target_deg"],round(sim[-1],2),round(a[-1],2),round(end-start,3),len(t),round(before,2),
                        round(max(0,before-min(a)),2),round(max(abs(g)),3),
                        round(t[moving[0]],3) if len(moving) else "",
                        round(t[dropped[0]],3) if len(dropped) else ""])
        for ax,y in zip(axes[:,col], [a,g]):
            if ax == axes[0,col] and (trial == 1 or col==2 and trial==2):
                ax.plot(t,sim,color="black",ls=":",label=f'Model {cue["target_deg"]}°')
            ax.plot(t,y,'.-',ms=3,label=f'Hold {trial}',alpha=.8)
    axes[0,col].set_title(['Earlier holds','Longer holds','Changed first target'][col])
    axes[1,col].set_xlabel('Seconds after SDK raise returned')
    axes[0,col].legend(ncol=2)
axes[0,0].set_ylabel('Head encoder (°)')
axes[1,0].set_ylabel('Gyro Y (rad/s)')
fig.suptitle('Head motion before any lower command · robot docked')
fig.savefig(root/'hold-overlay.png',dpi=170)
with (root/'hold-summary.csv').open('w') as f:
    w=csv.writer(f)
    w.writerow(['session','hold','animating_packets','head_in_pos_packets','target_deg','sim_last_deg','last_deg','wait_s','packets','start_deg','max_drop_deg','peak_abs_gyro_rad_s','gyro_onset_s','encoder_2deg_s'])
    w.writerows(summary)

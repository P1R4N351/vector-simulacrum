"""Freeze a steady encoder-to-yaw gain and an apparent time offset on the first pair."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
def read(name):
    with (ROOT/name).open() as f:
        return list(csv.DictReader(f))
def save(name, rows):
    with (ROOT/name).open("w") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

def load_trials(folder):
    events = read(folder/"events.csv")
    rows = read(folder/"states.csv")
    replays = read(folder/"predictions.csv")
    starts = [float(r["time_s"]) for r in events if r["event"] == "spin"]
    ends = [float(r["time_s"]) for r in events if r["event"] == "stop"]
    trials = []
    for i,(start,end) in enumerate(zip(starts, ends)):
        sample = [s for s in rows if start-.3 <= float(s["time_s"]) < end+.60]
        def vector(key):
            return np.array([float(s[key]) for s in sample])
        head = vector("head_rad") + np.deg2rad(-.66)
        yaw = np.sin(head)*vector("gyro_x") + np.cos(head)*vector("gyro_z")
        t = vector("time_s") - start
        diff = vector("right_mm_s") - vector("left_mm_s")
        sim = [s for s in replays if int(s["turn"]) == i+1]
        def model(key):
            return np.interp(t,[float(s["t_s"]) for s in sim],[float(s[key]) for s in sim])
        duration = end-start
        masks = {"acceleration":(0<=t)&(t<duration/2), "steady":(duration/2<=t)&(t<duration),
                 "braking":(duration<=t)&(t<duration+.30)}
        trials.append((t, yaw, diff, model("right_mm_s")-model("left_mm_s"), model("yaw_rad_s"), masks, duration))
    assert len(trials)==4
    return trials
trials = load_trials(Path("."))

# Give each training spin equal weight in its mean yaw and differential speed.
def gain(speed_index, yaw_index):
    x,y = [],[]
    for trial in trials[:2]:
        mask=trial[5]["steady"]
        x.append(trial[speed_index][mask].mean())
        y.append(trial[yaw_index][mask].mean())
    return np.dot(x,y)/np.dot(x,x)
g = gain(2,1)
g_model = gain(3,4)
def at_delay(trial, ms):
    t, _, diff = trial[:3]
    return g*np.interp(t-ms/1000,t,diff)
def transient_mse(trial, ms):
    mask = trial[5]["acceleration"] | trial[5]["braking"]
    return np.mean((at_delay(trial,ms)[mask]-trial[1][mask])**2)
grid = [{"delay_ms":ms,"train_transient_mse":np.mean([transient_mse(s,ms) for s in trials[:2]])}
        for ms in range(-120,121,5)]
lag = min(grid,key=lambda row:row["train_transient_mse"])["delay_ms"]
save("encoder-offset-grid.csv",grid)
import sys
external = "--fresh" in sys.argv
outdir = ROOT/"speed-holdout" if external else ROOT
test_trials = load_trials(Path("speed-holdout")) if external else trials[2:]
scores,trace = [],[]
fig,axes = plt.subplots(2,len(test_trials),figsize=(4.8*len(test_trials),6),sharex="col",layout="constrained")
for col,trial in enumerate(test_trials):
    turn_id = col+1 if external else col+3
    t,yaw,diff,sim_diff,sim_yaw,masks,duration = trial
    predictions = {"Frozen physics":sim_yaw,"Model gain × measured wheels":g_model*diff,
                   "Workshop gain × model wheels":g*sim_diff,"Workshop gain × measured wheels":g*diff,
                   "Workshop gain with offset":at_delay(trial,lag)}
    for model,pred in predictions.items():
        for phase,mask in masks.items():
            err = pred[mask]-yaw[mask]
            scores.append(dict(turn=turn_id,model=model,phase=phase,packets=sum(mask),
                               yaw_rmse_rad_s=np.sqrt(np.mean(err**2)),
                               steady_error_pct=(100*err.mean()/yaw[mask].mean() if phase=="steady" else "")))
    trace.extend(dict(turn=turn_id,t_s=x,yaw=yaw[j],**{k:v[j] for k,v in predictions.items()}) for j,x in enumerate(t))
    ax = axes[0,col]
    ax.plot(t,yaw,'k.',label="Measured gyro")
    for label in ["Frozen physics","Workshop gain × measured wheels","Workshop gain with offset"]:
        ax.plot(t,predictions[label],label=label,linewidth=1.4)
    axes[1,col].plot(t,diff,".-",label="Measured")
    axes[1,col].plot(t,sim_diff,"--",label="Simulated")
    for axis in axes[:,col]:axis.axvline(duration,color="grey",lw=.8)
    ax.set_title((["60 CCW","60 CW","150 CW","150 CCW"] if external else ["Held-out clockwise","Held-out counterclockwise"])[col])
    axes[1,col].set_xlabel("Seconds from command")
axes[0,0].set_ylabel("Chassis-axis yaw (rad/s)")
axes[1,0].set_ylabel("Right − left wheel speed (mm/s)")
axes[0,0].legend(fontsize=7);axes[1,0].legend(fontsize=8)
fig.savefig(outdir/"encoder-transfer.png",dpi=165)
save(outdir/"encoder-transfer-scores.csv",scores)
save(outdir/"encoder-transfer-traces.csv",trace)
print("physical gain",g,"model gain",g_model,"apparent delay ms",lag)
for model in predictions:
    print(model,[(s["turn"],s["phase"],round(s["yaw_rmse_rad_s"],3),round(s["steady_error_pct"],2) if s["steady_error_pct"]!="" else "") for s in scores if s["model"]==model])

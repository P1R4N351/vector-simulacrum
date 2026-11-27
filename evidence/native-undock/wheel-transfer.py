"""Score SDK-receipt and calibration-aligned yaw responses at physical sample times."""
import csv
import json
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

root = Path(sys.argv[1])
runs = {k: np.array(v) for k,v in json.loads((root/"input.json").read_text()).items()}
sim = {}
for scene in ("stock","driven"):
    data = np.genfromtxt(root/(scene+".transfer"), delimiter=",", names=True, dtype=None, encoding="utf8")
    for trial in runs:
        for mode in ("command","command-linear","command-immediate-linear","hold","linear"):
            sim[scene,trial,mode] = data[(data["trial"]==trial)&(data["mode"]==mode)]

def sample(scene, trial, mode, advance):
    row = runs[trial]
    mask = (13.7 <= row[:,0]) & (row[:,0] <= min(16,row[-1,0]-.15))
    t, left, right, head, _, gz = row[mask].T
    s = sim[scene,trial,mode]
    return t, np.degrees(gz), np.interp(t+advance,s["t"],s["gyro"]), (right-left)/2, (
        np.interp(t+advance,s["t"],s["right"]-s["left"])/2)
def rms(x):
    return np.mean(np.square(x))**.5
def rolling(trial, advance):
    row = runs[trial]
    t, y, _, wd, _ = sample("driven", trial, "linear", 0)
    # Nominal 46 mm lateral spacing from stock steering. Both inputs use SDK receipt time.
    speed = np.interp(t+advance,row[:,0],row[:,2]-row[:,1])
    projection = np.cos(np.interp(t,row[:,0],row[:,3]))
    return t, y, np.degrees(speed/46)*projection, wd, speed/2

def loss(advance, predictor):
    errors = []
    for trial in "BC":
        _, y, p, _, _ = predictor(trial,advance)
        errors.append(rms(p-y))
    return np.mean(errors)
lags = np.arange(-.15,.151,.005)
advance = min(lags, key=lambda d:loss(d,lambda trial,a:sample("driven",trial,"linear",a)))
rolling_advance = min(lags, key=lambda d:loss(d,rolling))
fig,axes=plt.subplots(2,3,figsize=(12,6),layout="constrained",sharex=True)
with open("wheel-transfer-scores.csv","w") as file:
    w=csv.writer(file,lineterminator="\n")
    w.writerow(["trial","scene","mode","advance_s","n","gyro_rmse_dps","peak_sim_dps","peak_real_dps","diff_rmse_mmps"])
    for trial in runs:
        for scene in ("stock","driven"):
            for mode in ("command","command-linear","command-immediate-linear","hold","linear"):
                for lag in ([0,advance] if scene=="driven" and mode=="linear" else [0]):
                    t,y,p,wd,pd=sample(scene,trial,mode,lag)
                    w.writerow([trial,scene,mode,round(lag,4),len(t),rms(y-p),max(abs(p)),max(abs(y)),rms(wd-pd)])
        for lag in [0,rolling_advance]:
            t,y,p,wd,pd=rolling(trial,lag)
            w.writerow([trial,"kinematic","rolling",round(lag,4),len(t),rms(y-p),max(abs(p)),max(abs(y)),rms(wd-pd)])
        if trial not in "DEF":
            continue
        top,bot=axes[:, "DEF".index(trial)]
        for axis,measured in ((top,y),(bot,wd)):
            axis.plot(t,measured,"ko-",lw=1.4,ms=3,label="Physical")
        for mode,lag,label in (("command-linear",0,"Interpolated command"),("linear",0,"Shaft replay"),
                               ("linear",advance,"Shaft, B/C timing")):
            t,y,p,wd,pd=sample("driven",trial,mode,lag)
            top.plot(t,p,label=label)
            bot.plot(t,pd,label=label)
        _,_,p,_,_=rolling(trial,rolling_advance)
        top.plot(t,p,":",lw=1.4,label="Rolling, B/C timing")
        top.set_title("Session "+trial)
        bot.set_xlabel("Seconds from undock request")
axes[0,0].set_ylabel("Head gyro Z (°/s)")
axes[1,0].set_ylabel("Half wheel difference (mm/s)")
axes[0,0].legend(fontsize=8)
fig.savefig("wheel-transfer.png",dpi=150)
print("B/C advance (ms)", round(advance*1000))

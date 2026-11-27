"""Compare pinned WireOS head targets to SDK traces on their grant clocks."""
import csv
import hashlib
import io
import json
from pathlib import Path
import tarfile
from urllib.request import urlopen

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
root = here.parents[1]
url = ("https://raw.githubusercontent.com/os-vector/wire-os-externals/"
       "6aec7f1ddf341a898692768650fe945e334fb865/animation-assets/animations/anim_gotosleep_wakeup_01.tar")
with urlopen(url, timeout=30) as response:
    payload = response.read()
assert hashlib.sha256(payload).hexdigest() == "5eb484bf0eb5ea4f4def6abdd7f7b236ea6ae7d840eb2cff317b1ee4edacb588"
with tarfile.open(fileobj=io.BytesIO(payload)) as archive:
    frames = json.load(archive.extractfile("anim_gotosleep_wakeup_01.json"))["anim_gotosleep_wakeup_01"]
keys = [f for f in frames if f["Name"] == "HeadAngleKeyFrame"]
times, angles = [], []
for k in keys:
    start = k["triggerTime_ms"] / 1000
    end = start + k["durationTime_ms"] / 1000
    if not times or start > times[-1] + 1e-6:
        times.append(start)
        angles.append(angles[-1] if angles else k["angle_deg"])
    times.append(end)
    angles.append(k["angle_deg"])

runs = []
for i, name in enumerate(["pilot", "session-1", "session-2", "calm-cross-1", "calm-cross-2"]):
    folder = root/"docked-head-20261126/grants" if i < 3 else here.parent/"blind-grants"
    with (folder/(name+".csv")).open() as f:
        rows = list(csv.DictReader(f))
    grant = next(float(r["receipt_s"]) for r in rows if r["event"] == "granted")
    states = [r for r in rows if r["event"] == "state" and 0 <= float(r["receipt_s"])-grant <= 5.2]
    runs.append((name, np.array([float(r["receipt_s"])-grant for r in states]),
                 np.array([float(r["head_deg"]) for r in states])))

def error(t, x, delay):
    return np.sqrt(np.mean((np.interp(t-delay, times, angles)-x)**2))

delays = np.arange(-300, 801, 5)/1000
delay = min(delays, key=lambda d: sum(error(t,x,d) for _,t,x in runs[:2]))
fig, axes = plt.subplots(2, 1, figsize=(9,6), sharex=True, layout="constrained")
with (here/"scores.csv").open("w") as f:
    w = csv.writer(f, lineterminator="\n")
    w.writerow(["run","role","n","zero_delay_rmse_deg","shared_delay_s","shared_delay_rmse_deg"])
    for i,(name,t,x) in enumerate(runs):
        axes[int(i>=3)].plot(t,x, label=name, alpha=.8)
        w.writerow([name,"train" if i<2 else "held-out",len(t),error(t,x,0),delay,error(t,x,delay)])
for ax, title in zip(axes, ["Workshop A", "Workshop B · both recordings held out"]):
    ax.plot(np.array(times)+delay,angles,"k.--",label="WireOS head target ramp")
    ax.set(ylabel="Head angle (°)", title=title, xlim=(0,5.2))
    ax.legend(ncol=2,fontsize=8)
axes[-1].set_xlabel("Seconds after recorded SDK control grant")
fig.savefig(here/"comparison.png",dpi=165)

fig, axes = plt.subplots(1,3,figsize=(12,3.4),layout="constrained",sharey=True)
with (here/"interrupted.csv").open("w") as f:
    w = csv.writer(f,lineterminator="\n")
    w.writerow(["run","window_start_s","window_end_s","n","wake_rmse_deg","hold_rmse_deg"])
    for i,ax in enumerate(axes,1):
        with (here.parent/"wake-raise"/f"wake-raise-{i}.csv").open() as src:
            rows = list(csv.DictReader(src))
        events = {r["event"]:float(r["receipt_s"]) for r in rows if r["event"]!="state"}
        grant = events["granted"]
        success = events["raise_return_0"]-grant
        states = [r for r in rows if r["event"]=="state" and grant<=float(r["receipt_s"])<events["restore"]]
        t = np.array([float(r["receipt_s"])-grant for r in states])
        x = np.array([float(r["head_deg"]) for r in states])
        wake = np.interp(t-delay,times,angles)
        valid = (t>=success+.5)&(t<8)
        w.writerow([i,success+.5,8,valid.sum(),error(t[valid],x[valid],delay),np.sqrt(np.mean((22-x[valid])**2))])
        ax.plot(t,x,label="Measured");ax.plot(t,wake,"--",label="Wake curve");ax.axhline(22,color=".6",ls=":",label="SDK target")
        ax.axvspan(success+.5,8,color=".7",alpha=.2,label="Scored window")
        ax.set(title=f"Raise at {events['raise']-grant:.1f} s",xlabel="Seconds after grant")
axes[0].set_ylabel("Head angle (°)")
axes[0].legend(fontsize=8)
fig.savefig(here/"interrupted.png",dpi=165)

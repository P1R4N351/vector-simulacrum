"""Fit one post-acceptance delay on workshop strokes; evaluate full held-out unit 3 trajectories."""
import csv
import json
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
timing = here.parent
sys.path.insert(0,str(timing))
from analyze import GyroReplay, params, recording, replay

moves = list(csv.DictReader((here/"state-relative.csv").read_text().splitlines()))
fit = [m for m in moves if m["robot"] in ("Unit 1","Unit 2")]
delay = np.median([(float(m["physical_lo_ms"])+float(m["physical_hi_ms"])
                    -float(m["sim_lo_ms"])-float(m["sim_hi_ms"]))/2000 for m in fit])
values = dict(params.values(params.load(model="vector_2")),**replay.OPEN_TABLE)
rows = []
fig,axes = plt.subplots(2,2,figsize=(9,6),layout="constrained")
for col,(label,path) in enumerate([("Unit 3 archive",timing/"repeat/unit3.jsonl"),("Unit 3 new",here/"unit3.jsonl")]):
    states = [s for s in map(json.loads,path.read_text().splitlines()) if s["k"]=="s"]
    for m in (r for r in moves if r["robot"]==label):
        start,target = float(m["command_s"]),float(m["target"])
        near = [s for s in states if -.22 < s["t"]-start < 1.3]
        t = np.array([s["t"]-start for s in near])
        bits = np.array([bool(s["status"] & 512) for s in near])
        idx = np.flatnonzero((t[1:]>=0) & bits[:-1] & ~bits[1:]).item()
        initial = replay._state_at(near,start)
        sim = GyroReplay(values=values,head_deg=np.degrees(initial["head"]),lift_mm=initial["lift"],noise_seed=0)
        # Log the actual status transition from the public control-tick execution.
        trace = []
        for tick in range(round(1.65/sim.dt)):
            if tick == round(.32/sim.dt): sim.command_head_angle(target)
            sim.step()
            trace.append((sim.tick*sim.dt-.32,sim._head_in_pos,sim.reported_now()))
        ts = np.array([s[0] for s in trace])
        sim_bits = np.array([s[1] for s in trace])
        i = np.flatnonzero(sim_bits[:-1] & ~sim_bits[1:]).item()
        sim_accept = ts[i+1]
        window = (t >= 0) & (t <= 1.1)
        row = dict(session=label, command_s=start,target=target,delay_ms=1000*delay)
        for ax,signal,measured,unit in zip(axes[:,col],("head_deg","pitch_speed"),
                (np.degrees([s["head"] for s in near]),-np.degrees([s["gyro"][1] for s in near])),("deg","dps")):
            predicted = [s[2][signal] for s in trace]
            for anchor,accept in (("early",t[idx]),("mid",np.mean(t[idx:idx+2])),("late",t[idx+1])):
                for variant,added in (("existing",0),("delayed",delay)):
                    pred = np.interp(t[window]-accept+sim_accept-added,ts,predicted)
                    row[f"{anchor}_{variant}_{unit}"] = np.sqrt(np.mean((pred-measured[window])**2))
            ax.plot([0,1],[row[f"mid_existing_{unit}"],row[f"mid_delayed_{unit}"]],"o-",alpha=.75,
                    color="#176d74" if target>np.degrees(initial["head"]) else "#aa5726")
            ax.set(xticks=[0,1],xticklabels=["Existing",f"+{delay*1000:.0f} ms"],xlim=(-.25,1.25),
                   ylabel="RMS error (°)" if unit=="deg" else "RMS error (°/s)")
        rows.append(row)
    axes[0,col].set_title(label)
fig.suptitle("Head motion aligned at observed status change · raises in teal, lowers in orange")
fig.savefig(here/"status-transfer.png",dpi=145)
with (here/"status-transfer.csv").open("w") as f:
    w=csv.DictWriter(f,fieldnames=rows[0],lineterminator="\n")
    w.writeheader()
    w.writerows(rows)
for group in ("Unit 3 archive","Unit 3 new"):
    group_rows=[r for r in rows if r["session"]==group]
    for unit in ("deg","dps"):
        for anchor in ("early","mid","late"):
            errs=np.array([[r[f"{anchor}_{k}_{unit}"] for k in ("existing","delayed")] for r in group_rows])
            print(group,unit,anchor,np.round(np.median(errs,axis=0),2),sum(errs[:,1]<errs[:,0]))
print("Frozen delay",delay)

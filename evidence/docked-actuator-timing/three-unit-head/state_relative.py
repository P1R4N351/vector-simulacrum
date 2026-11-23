"""Check the time from head command acceptance to motion using only within-stream differences."""
import csv
import json
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
timing = here.parent
sys.path.insert(0, str(timing))
from analyze import GyroReplay, params, recording, replay


class Replay(GyroReplay):
    def reported_now(self):
        return dict(super().reported_now(), settled=self._head_in_pos)


def interval(t, bits, speed):
    active = np.flatnonzero((t >= 0) & (abs(speed) > 10))
    falls = np.flatnonzero((t[1:] >= 0) & bits[:-1] & ~bits[1:])
    assert len(falls) == 1 and len(active) and active[0] > 0
    i, j = falls.item(), active[0]
    return t[j-1]-t[i+1], t[j]-t[i]


sessions = [("Unit 1",timing/"repeat/unit1.jsonl","Unit 1"),("Unit 2",timing/"unit2.jsonl","Unit 2"),
            ("Unit 3 archive",timing/"repeat/unit3.jsonl","Unit 3"),("Unit 3 new",here/"unit3.jsonl",None)]
moves = list(csv.DictReader((timing/"repeat/moves.csv").read_text().splitlines()))
rows = []
fig, axes = plt.subplots(1,4,figsize=(12,5),sharex=True,layout="constrained")
for (name,path,unit), ax in zip(sessions,axes):
    data = [json.loads(line) for line in path.read_text().splitlines()]
    states = [s for s in data if s["k"]=="s"]
    commands = [(float(m["command_s"]),45 if m["direction"]=="up" else -22)
                for m in moves if m["source"]==unit and m["joint"]=="head"] if unit else [
                    (r["t"],r["target"]) for r in data if r["k"]=="command"]
    values = dict(params.values(params.load(model="vector_2" if name.startswith("Unit 3") else "vector_1")),**replay.OPEN_TABLE)
    for i,(cmd,target) in enumerate(commands):
        near = [s for s in states if -.2 < s["t"]-cmd < 1.3]
        t = np.array([s["t"]-cmd for s in near])
        bits = np.array([bool(s["status"] & 512) for s in near])
        lo,hi = interval(t,bits,np.degrees([s["gyro"][1] for s in near]))
        initial = replay._state_at(near,cmd)
        sim = Replay(values=values,head_deg=np.degrees(initial["head"]),lift_mm=initial["lift"],noise_seed=0)
        tr = replay._run(sim,recording.Segment("head","head",0,1.3,target=target),.32)
        ts = (np.arange(len(tr))+1)*sim.dt-.32
        sl,sh = interval(ts,np.array([s["settled"] for s in tr]),np.array([s["pitch_speed"] for s in tr]))
        rows.append(dict(robot=name,command_s=cmd,target=target,physical_lo_ms=lo*1000,physical_hi_ms=hi*1000,
                         sim_lo_ms=sl*1000,sim_hi_ms=sh*1000,gap_ms=1000*max(0,lo-sh,sl-hi)))
        ax.plot([1000*lo,1000*hi],[i,i],"|-",color="#227478")
        ax.plot([1000*sl,1000*sh],[i,i],"|-",color="#ba6625",lw=4)
    ax.set(title=name,yticks=range(len(commands)),yticklabels=[f"{i+1} · {p[1]}°" for i,p in enumerate(commands)])
    ax.axvline(0,color=".7",lw=.7)
    ax.invert_yaxis()
fig.suptitle("Time from HEAD_IN_POS clearing to pitch gyro crossing 10°/s")
fig.supxlabel("Milliseconds · teal SDK brackets, orange MuJoCo brackets")
fig.savefig(here/"state-relative.png",dpi=150)
with (here/"state-relative.csv").open("w") as f:
    w=csv.DictWriter(f,fieldnames=rows[0],lineterminator="\n")
    w.writeheader()
    w.writerows(rows)
for name,_,_ in sessions:
    subset=[r for r in rows if r["robot"]==name]
    print(name,len(subset),sum(r["gap_ms"]<1e-6 for r in subset),round(np.median([r["gap_ms"] for r in subset]),1))

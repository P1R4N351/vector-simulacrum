"""Evaluate frozen lift timing on stationary stops using host receipt timestamps."""
import csv
import gzip
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from simulacrum import params
from simulacrum.robot import VectorSim

here = Path(__file__).resolve().parent
with gzip.open(here / "stop-control.jsonl.gz", "rt") as stream:
    records = [json.loads(line) for line in stream]
states = [r for r in records if r["kind"] == "state"]
responses = {r["n"]: r["t"] for r in records if r["kind"] == "response"}
lag = .11552706003188984  # frozen in status_lag.py
fig, axes = plt.subplots(4, 1, figsize=(8, 6), sharex=True, layout="constrained")
results = []
for ax, event in zip(axes, (r for r in records if r.get("action") == "stop")):
    rows = [s for s in states if -.1 < s["t"] - event["t"] < .45]
    t = np.array([s["t"] - event["t"] for s in rows])
    bit = np.array([bool(s["status"] & 256) for s in rows])
    clear = np.where(~bit)[0]
    first, last = clear[0], clear[-1]
    assert first > 0 and last + 1 < len(bit) and not bit[first:last+1].any()
    sim = VectorSim(params.values(params.load(model="vector_2")), lift_mm=rows[0]["lift"])
    for _ in range(round(1.0 / sim.dt)):
        sim.step()
    assert sim.stop_all_motors()
    ticks = 0
    while ticks < 100 and not sim._lift_in_pos:
        sim.step()
        ticks += 1
    assert sim._lift_in_pos
    dwell = ticks * sim.dt
    # Counterfactual: deliver the stationary hold directly to the simulated servo.
    direct = VectorSim(params.values(params.load(model="vector_2")), lift_mm=rows[0]["lift"])
    direct.step(round(1.0 / direct.dt))
    assert direct.stop_all_motors()
    for slot in direct._delay:
        slot[3] = direct._lift_target
    direct_ticks = 0
    while direct_ticks < 100 and not direct._lift_in_pos:
        direct.step()
        direct_ticks += 1
    assert direct._lift_in_pos
    ax.plot(t*1000, bit, 'ko', label="SDK sample")
    for anchor, shift, color in [('dispatch', 0, 'C0'), ('response', responses[event["n"]]-event["t"], 'C1'),
                                 ('direct hold + dispatch', 0, 'C3'), ('direct hold + response', responses[event["n"]]-event["t"], 'C2')]:
        start = lag + shift
        end = start + (direct_ticks * direct.dt if anchor.startswith("direct") else dwell)
        pred_clear = (t >= start) & (t < end)
        hits = np.sum(pred_clear & ~bit)
        false = np.sum(pred_clear & bit)
        results.append([event["n"]+1,anchor,shift,start,end,t[first-1],t[first],t[last],t[last+1],int(hits),len(clear),int(false)])
        ax.plot(1000*np.array([-.1,start,start,end,end,.45]), np.array([1,1,0,0,1,1])+(.06 if shift else -.06),color=color,label=anchor)
    ax.set(yticks=[0,1],yticklabels=['Clear','Set'],ylabel=f'Trial {event["n"]+1}')
axes[0].legend(ncol=3, loc='center right')
axes[-1].set_xlabel('Milliseconds from SDK stop call')
fig.savefig(here/'stop-clock.png', dpi=165)
with (here/'stop-clock.csv').open('w') as stream:
    w=csv.writer(stream)
    w.writerow('trial anchor shift_s predicted_clear_s predicted_set_s last_set_s first_clear_s last_clear_s first_set_s clear_hits observed_clear false_clear'.split())
    w.writerows(results)

"""Replay delivered radio packets and compare ordered events from the same physical run."""
import csv
from pathlib import Path
import sys
import numpy as np
import matplotlib.pyplot as plt

from decode import decode
here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[1]))
from simulacrum.cube import MovementListener, TapListener, UpAxisListener, engine_samples

actions = list(csv.DictReader((here / "action-times.csv").open()))
start = float(next(e["pod_unix_s"] for e in actions if e["action"] == "pickup"))
packets, _ = decode(here / "pickup.tsv")
tap, move, up = TapListener(), MovementListener(), UpAxisListener()
for host, _, count, slots in packets:
    t = host * 1e-9 - start
    payload = bytes([2, count]) + slots
    tap.receive(t, payload)
    up.receive(t, payload)
    for accel in engine_samples(payload):
        move.update(t, accel)

with (here / "sdk-cube-events.csv").open() as f:
    events = list(csv.DictReader(f))
series = {"Packet tap": tap.events, "SDK tap": [], "Listener move": [], "SDK move": [],
          "Listener stop": [], "SDK stop": [], "Listener axis": [t for t,_ in up.events], "SDK axis": []}
for t,on in move.events:
    series["Listener move" if on else "Listener stop"].append(t)
for ev in events:
    label = {"object_tapped":"SDK tap","object_stopped_moving":"SDK stop","object_moved":"SDK move",
             "object_up_axis_changed":"SDK axis"}[ev["event"]]
    series[label].append(float(ev["pod_unix_s"])-start)
fig,ax=plt.subplots(figsize=(10,4))
for i,(name,times) in enumerate(series.items()):
    ax.scatter(times,[i]*len(times),color="teal" if name.startswith("SDK") else "darkorange")
ax.set_yticks(range(len(series)),list(series))
ax.set_xlabel("Seconds from pickup command")
ax.invert_yaxis()
fig.tight_layout()
fig.savefig(here/"event-replay.png",dpi=150)
with (here/"event-replay.csv").open("w") as out:
    w=csv.writer(out,lineterminator="\n")
    w.writerow(["stream","seconds_from_pickup"])
    for name,times in series.items():
        w.writerows((name,t) for t in times)
print(series)
if len(tap.events)==len(series["SDK tap"]):
    print("Ordered tap offsets (ms)", np.round(1000*(np.array(series["SDK tap"])-tap.events),1))

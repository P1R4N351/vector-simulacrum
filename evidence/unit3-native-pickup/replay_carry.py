"""Replay observed shaft speeds with the frozen tread inset and SDK carried-cube placement."""
import csv
import gzip
import json
import sys
from pathlib import Path
from xml.etree import ElementTree

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[1]))
from replay_measured_turn import run
from simulacrum import mjcf, params

values = params.values(params.load(model="vector_2"))


def fixture(root, loaded):
    for suffix in ("fl", "fr", "rl", "rr"):
        patch = root.find(f".//body[@name='patch_{suffix}']")
        position = list(map(float, patch.get("pos").split()))
        axle = root.find(f".//body[@name='wheel_{suffix}']")
        position[0] = float(axle.get("pos").split()[0]) + (.00375 if suffix.startswith("r") else -.00375)
        patch.set("pos", mjcf.vec(position))
    if loaded:
        root.find(".//body[@name='lift_fork']").append(ElementTree.fromstring(mjcf._carried_cube_xml(values)))


def main():
    with gzip.open(here / "rotation-verified.jsonl.gz", "rt") as source:
        events = list(map(json.loads, source))
    states = [e for e in events if e.get("name") == "robot_state"]
    scores = []
    fig, axes = plt.subplots(3, 3, figsize=(12, 8), sharey="col")
    for i, e in enumerate(events):
        action = e.get("action", "")
        if not (e["kind"] == "begin" and " turn " in action or
                e["kind"] == "pulse_begin" and action.endswith(" reverse")):
            continue
        condition, motion = action.split(" ", 1)
        trial = next(x["number"] for x in reversed(events[:i]) if x["kind"] == "trial")
        reversal = motion == "reverse"
        end = next(x["t"] for x in events[i+1:] if x["kind"] == ("pulse_end" if reversal else "end")
                   and x.get("action") == (condition+" forward2" if reversal else action))
        start = next(x["t"] for x in reversed(events[:i+1]) if x["kind"] == "begin"
                     and x.get("action") == condition+" turn outward")
        selected = [x for x in states if start-.35 <= x["t"] <= end+(.45 if reversal else 0)]
        t = np.array([x["t"]-e["t"] for x in selected])
        d = [x["data"] for x in selected]
        wheels = np.array([[v.get(k, 0) for k in ("left_wheel_speed_mmps","right_wheel_speed_mmps")] for v in d])
        gyro = np.degrees([v["gyro"].get("z",0) for v in d])
        pose = [{"head":d[0]["head_angle_rad"],"lift":d[0]["lift_height_mm"]}]
        ax = axes[2 if trial == 4 else int(condition=="empty"),
                  2 if reversal else int(motion=="turn back")]
        window = t >= -.2
        ax.plot(t[window],gyro[window],"k.-",label="Measured")
        for loaded in (False,True):
            sim = run(pose,t,wheels,40.5,configure=lambda root: fixture(root,loaded))
            error = sim[:,0]-gyro
            active = t>=0
            scores.append([trial,condition,motion,int(loaded),np.sqrt(np.mean(error[active]**2)),
                           np.max(abs(sim[active,1:]-wheels[active]))])
            ax.plot(t[window],sim[window,0],label="50 g carried" if loaded else "Empty",ls="--" if loaded else ":")
        ax.set_title(f"Trial {trial}, {condition}, {motion}")
        ax.set_xlabel("Time from action (s)")
    for ax in axes[:,0]:
        ax.set_ylabel("Head gyro Z (°/s)")
    axes[0,0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(here/"replay-carry.png",dpi=150)
    with (here/"replay-carry.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["trial","observed","action","sim_loaded","gyro_rmse_dps","max_wheel_input_error_mmps"])
        w.writerows(scores)


if __name__ == "__main__":
    main()

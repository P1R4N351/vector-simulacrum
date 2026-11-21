"""Replay the nine Run G spins in one table frame and publish the drift comparison."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from replay_measured_turn import run

ROOT = Path(__file__).resolve().parent / "evidence"
DATA = ROOT / "endurance-drift"


def relative_center(xy, yaw):
    # Unloaded hardware odometry tracks a point 25 mm aft of the reported origin.
    center = xy - 25 * np.column_stack((np.cos(yaw), np.sin(yaw)))
    c, s = np.cos(yaw[0]), np.sin(yaw[0])
    return (center - center[0]) @ np.array([[c, -s], [s, c]])


def replay(rows, t, wheels):
    clock, out = run(rows, t, wheels, 42, dense=True, include_pose=True)
    yaw = np.unwrap(out[:, 5])
    field = np.column_stack((relative_center(out[:, 3:5], yaw), np.degrees(yaw-yaw[0])))
    return np.column_stack([np.interp(t, clock, v) for v in field.T])


def main():
    events = list(map(json.loads, (DATA / "run-g.jsonl").read_text().splitlines()))
    states = [r for r in events if r["k"] == "s"]
    cmds = [r for r in events if r.get("name") == "wheels"]
    rows = [r for r in states if 44.90 <= r["t"] <= 92.1]
    t = np.array([r["t"] for r in rows])
    wheels = np.array([[r["wl"], r["wr"]] for r in rows])
    yaw = np.unwrap([r["yaw"] for r in rows])
    xy = np.array([r["pose"][:2] for r in rows])
    obs = np.column_stack((relative_center(xy, yaw), np.degrees(yaw-yaw[0])))
    original = replay(rows, t, wheels)
    # Host delivered a burst after the ninth stop. Redistribute those samples
    # at uniform intervals between the last timely sample and end of backlog.
    repaired = t.copy()
    lo = np.argmax(np.diff(t))
    hi = np.searchsorted(t, 91.49)
    repaired[lo:hi+1] = np.linspace(t[lo], t[hi], hi-lo+1)
    nominal = replay(rows, repaired, wheels)
    diff = (wheels[:, 1] - wheels[:, 0])/2
    symmetric = replay(rows, repaired, np.column_stack((-diff, diff)))
    ends = [np.searchsorted(t, c["t"])-1 for c in cmds[1:]] + [len(t)-1]
    marks = [0, *ends]
    heading = yaw-yaw[0]
    velocity = wheels.mean(axis=1)[:, None]*np.column_stack((np.cos(heading), np.sin(heading)))
    increments = .5*(velocity[:-1]+velocity[1:])*np.diff(repaired)[:, None]
    kinematic = np.vstack((np.zeros(2), np.cumsum(increments, axis=0)))
    datasets = [obs, original, nominal, symmetric]
    names = ["SDK odometry", "Replay · arrival times", "Replay · paced backlog", "Replay · balanced wheels"]
    colors = ["#172e50", "#cb5642", "#298673", "#9173ab"]
    with (DATA/"turns.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["turn","direction","end_s","wheel_mean_integral_mm",
                         *[f"{tag}_{v}" for tag in ("sdk","arrival","paced","balanced")
                           for v in ("dx_mm","dy_mm","net_x_mm","net_y_mm","turn_deg")]])
        for i,(a,b) in enumerate(zip(marks,marks[1:])):
            values = []
            for series in datasets:
                delta = series[b]-series[a]
                values.extend([*delta[:2],*series[b,:2],delta[2]])
            distance = np.trapezoid(wheels[a:b+1].mean(axis=1),repaired[a:b+1])
            writer.writerow([i+1, "CCW" if "_ccw_" in cmds[i]["label"] else "CW", t[b],
                             round(distance,3), *np.round(values,3)])
    fig, ax = plt.subplots(1,2, figsize=(12,5),layout="constrained")
    for data,name,color in zip(datasets,names,colors):
        ax[0].plot(*data[marks,:2].T,".-",color=color,label=name,alpha=.88)
        ax[1].plot(range(1,10),np.linalg.norm(np.diff(data[marks,:2],axis=0),axis=1),"o-",ms=4,color=color)
    ax[0].plot(*kinematic[marks].T, ":", color="#a37516", label="Wheel mean + SDK heading")
    ax[1].plot(range(1,10),np.linalg.norm(np.diff(kinematic[marks],axis=0),axis=1),":",color="#a37516")
    for i,k in enumerate(ends,1):
        ax[0].annotate(str(i),obs[k,:2],xytext=(2,4 if i%2 else -9),textcoords="offset points",fontsize=8)
    ax[0].set(aspect="equal",xlabel="Forward (mm)",ylabel="Left (mm)",title="Resting drive-center positions")
    ax[0].legend(fontsize=8)
    ax[1].set(xlabel="Spin number  (odd CCW, even CW)",ylabel="Distance between rests (mm)",xticks=range(1,10),title="Translation per turn, including braking")
    for a in ax:a.grid(alpha=.2)
    fig.suptitle("Vector 2.0  |  Nine alternating spins on the same surface")
    fig.savefig(ROOT/"physics-plots/endurance-drift.png",dpi=160)
    plt.close(fig)

    # Register the robot log to recording time using the SDK connection log's
    # control_lost_event and first connect (hardware clock used for differences).
    # Host connect emitted at t=.0025, stderr connect at 17:20:00.914.
    loss = 91.419 + .0025
    cliff = loss + (1611507-1583220)/1000
    fig, ax = plt.subplots(2,1,figsize=(10,6),layout="constrained")
    ax[0].plot(t,obs[:,2],label="SDK yaw",color=colors[0])
    for series,name,color in zip(datasets[1:3],names[1:3],colors[1:3]):
        ax[0].plot(t,series[:,2],label=name,color=color,ls="--",lw=1.25)
    ax[0].set(ylabel="Unwrapped yaw (degrees)",xlabel="Run time (s)",title="Whole-run heading in the initial frame")
    ax[0].legend(fontsize=8)
    ts=np.array([r["t"] for r in states])
    ax[1].scatter(ts,[int(bool(r["status"] & 0x4000)) for r in states],s=4,label="SDK cliff bit",color=colors[0])
    for when,label,y in ((loss,"Sleep / control lost",.72),(96.6636,"Abort logged",.42),
                         (ts[-1],"Last SDK sample",.16),(cliff,"Firmware cliff stop",.9)):
        ax[1].axvline(when,color="#666",lw=1)
        ax[1].annotate(label,(when,y),xytext=(3,0),textcoords="offset points",fontsize=8,rotation=22)
    ax[1].set(xlim=(85,126),ylim=(-.1,1.3),yticks=[0,1],ylabel="SDK cliff bit",xlabel="Run time (s)",title="Last turn, control handoff and subsequent cliff event")
    ax[1].legend(loc="upper left",fontsize=8)
    for a in ax:a.grid(alpha=.2)
    fig.savefig(ROOT/"physics-plots/endurance-timeline.png",dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()

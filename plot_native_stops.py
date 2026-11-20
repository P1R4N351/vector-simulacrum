"""Reproduce the wheel-edge/gyro comparison for the four Vector 2.0 stops."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

EVIDENCE = Path(__file__).resolve().parent / "evidence"
DATA = EVIDENCE / "hw-rec-20261119"


def analyze(rows, direction):
    def column(key):
        return np.array([float(row[key]) for row in rows])
    stamp = column("stamp_ms")
    counts = np.column_stack([column("left_count"), column("right_count")])
    speeds = np.column_stack([column("left_mmps"), column("right_mmps")])
    edge = np.flatnonzero(np.any(np.diff(counts, axis=0), axis=1))[-1] + 1
    t = stamp - stamp[edge]
    assert np.array_equal(t, column("ms_from_last_edge"))
    zero = np.flatnonzero(np.any(speeds, axis=1))[-1] + 1
    assert zero >= edge and t[-1] >= 500
    # The internal body frame sometimes has timestamp 0 (no new FIFO sample).
    # Use the published head gyro and measured head angle, on every control tick.
    yaw = direction * np.degrees(column("head_gz") / np.cos(column("head_rad")))
    window = (t >= -100) & (t <= 150)
    assert np.all(np.diff(column("frame")[window]) == 1)
    valid = (column("imu_stamp_ms") > 0) & window
    assert np.median(abs(yaw[valid]-direction*np.degrees(column("body_gz")[valid]))) < .01
    for index in (edge, zero):
        assert valid[index] and abs(yaw[index]-direction*np.degrees(column("body_gz")[index])) < .01
    assert np.ptp(column("head_rad")[window]) < .001
    yaw -= np.median(yaw[t>=390])
    # Anchor the integral at the observed edge. Interpolate the endpoint exactly.
    def angle(begin, end):
        times = np.r_[begin,t[(t>begin)&(t<end)],end]
        return np.trapezoid(np.interp(times,t,yaw), times/1000)
    metrics = [int(t[zero]),yaw[edge],yaw[zero],angle(0,100),angle(t[zero],100),
               angle(17,117),np.percentile(abs(yaw[t>=390]),95)]
    return t, counts, speeds, yaw, zero, edge, metrics


def main():
    with (DATA/"unit2-native-stops.csv").open() as f:
        rows = list(csv.DictReader(f))
    turns = list(dict.fromkeys(row["turn"] for row in rows))
    fig, axes = plt.subplots(3,len(turns),figsize=(13,7.3),sharex=True,sharey="row")
    for i, turn in enumerate(turns):
        trial = [row for row in rows if row["turn"] == turn]
        direction = 1 if turn.startswith("ccw") else -1
        t, counts, speeds, yaw, zero, edge, metrics = analyze(trial, direction)
        print(turn, *(f"{v:.3f}" for v in metrics))
        view = (t >= -80) & (t <= 100)
        sdk = view & np.array([row["sdk_match"]=="1" for row in trial])
        ax, enc, rate = axes[:,i]
        ax.plot(t[view], yaw[view], color="#136d71", marker=".", ms=4)
        ax.scatter(t[sdk],yaw[sdk],s=40,facecolor="none",edgecolor="#252938",zorder=3,label="Also seen by SDK")
        # Raw shaft polarity is opposite between the left and right encoders.
        for j, (color, label) in enumerate([("#4c66a4","Left"),("#bd633e","Right")]):
            enc.step(t[view],direction*(counts[view,j]-counts[edge,j]),where="post",color=color,label=label)
            rate.plot(t[view],abs(speeds[view,j]),color=color)
        for axis in (ax,enc,rate):
            axis.axvline(0,color="#333",lw=1)
            axis.axvline(t[zero],color="#ad5a20",ls="--",lw=1)
            axis.grid(alpha=.15)
            axis.spines[["right","top"]].set_visible(False)
        ax.set_title(turn.replace("_"," ").upper())
    axes[0,0].set_ylabel("Turn-aligned yaw (°/s)")
    axes[1,0].set_ylabel("Counts relative to last edge")
    axes[2,0].set_ylabel("|Wheel speed| (mm/s)")
    axes[0,0].legend(fontsize=8)
    axes[1,0].legend(fontsize=8)
    fig.suptitle("Vector 2.0 braking on the robot clock", fontsize=17, weight="bold")
    fig.text(.5,.925,"Solid line: final detected shaft edge     •     Dashed line: both reported wheel speeds reach zero",ha="center")
    fig.supxlabel("Milliseconds from the final change in either shaft encoder")
    fig.subplots_adjust(left=.085,right=.985,top=.84,bottom=.105,hspace=.19,wspace=.13)
    fig.savefig(EVIDENCE/"physics-plots"/"native-wheel-stop.png",dpi=160)


if __name__=="__main__":
    main()

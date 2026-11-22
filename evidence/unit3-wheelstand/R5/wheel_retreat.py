"""Compare longitudinal pose travel and integrated wheel reports during two raises."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import cumulative_trapezoid

from pickup_onset import action_time, read

here = Path(__file__).resolve().parent
earlier = here.parents[1]/"unit3-cube-accelerometer"


def wheel_path(t, heading, wheel_time, left, right):
    speed = np.interp(t, wheel_time, (left + right)/2)
    return np.stack([cumulative_trapezoid(speed * direction, t, initial=0)
                     for direction in (np.cos(heading), np.sin(heading))])


def main():
    fig, axes = plt.subplots(2, 2, figsize=(9, 6), layout="constrained")
    rows = []
    for col, label in enumerate(("Earlier", "R5")):
        if label == "Earlier":
            path, robot = read(earlier/"pickup-path.csv"), read(earlier/"robot.csv")
            start = action_time(earlier/"action-times.csv", "pickup", "pod_unix_s")
            pt, ph, yaw = path["time_s"], path["lift_mm"], path["yaw_rad"]
            xy = [path["x_mm"], path["y_mm"]]
            wt, left, right = robot["pod_unix_s"] - start, robot["left_mmps"], robot["right_mmps"]
        else:
            path, robot = read(here/"approach.csv"), read(here/"state.csv.gz")
            start = action_time(here/"actions.csv", "pickup for inward transport", "sec")
            pt, ph, yaw = path["sec"], path["lift"], path["yaw"]
            xy = [path["x"], path["y"]]
            wt, left, right = robot["sec"] - start, robot["left"], robot["right"]
        baseline = np.median(ph[(pt>=0)&(pt<1)])
        def crossing(h):
            i = np.flatnonzero((pt>1)&(ph>h))[0]
            return np.interp(h, ph[i-1:i+1], pt[i-1:i+1])
        t = np.linspace(crossing(baseline + 1), crossing(np.max(ph[(pt>1)&(pt<4)])-1), 1001)
        heading = np.interp(t, pt, np.unwrap(yaw))
        forward = np.array([np.cos(heading[0]), np.sin(heading[0])])
        observed = np.array([np.interp(t,pt,v) for v in xy])
        observed -= observed[:, :1].copy()
        wheel = wheel_path(t,heading,wt,left,right)
        traces = [forward @ a for a in (observed,wheel)]
        for trace, name in zip(traces, ("SDK pose","Integrated wheels")):
            axes[0,col].plot(t-t[0],trace,label=name)
        axes[1,col].plot(t-t[0],traces[0]-traces[1])
        axes[0,col].set_title(label)
        for ax in axes[:,col]:
            ax.set_xlabel("Seconds from 1 mm rise")
        rows.append([label,t[0],t[-1],traces[0][-1],traces[1][-1],
                     max(abs(traces[0]-traces[1])),np.linalg.norm(observed[:,-1]-wheel[:,-1])])
    axes[0,0].legend()
    axes[0,0].set_ylabel("Forward displacement (mm)")
    axes[1,0].set_ylabel("Pose minus wheels (mm)")
    fig.savefig(here/"wheel-retreat.png",dpi=155)
    with (here/"wheel-retreat.csv").open("w") as f:
        w=csv.writer(f)
        w.writerow("run onset_s near_full_s pose_forward_mm wheel_forward_mm peak_forward_difference_mm end_xy_difference_mm".split())
        w.writerows(rows)


if __name__ == "__main__":
    main()

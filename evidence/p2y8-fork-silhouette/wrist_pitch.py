"""Fit only the pictured change of edge column with row; test on cube motion."""
import csv
import json
import sys
import matplotlib.pyplot as plt
import numpy as np
from geometry import HERE, silhouette

sys.path.insert(0, str(HERE.parent / "p2y8-printed-fork-contact"))
from reach import score

poses = json.loads((HERE / "low/poses.json").read_text())
picks = np.genfromtxt(HERE / "low/edges.csv", names=True, delimiter=",", encoding="utf8", dtype=None)
picks = picks[(picks["optics"] == "shelf") & (picks["reach_mm"] == 2)]


def optical(pose, pitch):
    mask = silhouette(pose["lift"], pose["head"], np.deg2rad(5.16), -.52, 2, pitch)
    points = picks[picks["trial"] == pose["trial"]]
    pred = np.array([500 + np.flatnonzero(mask[y, 500:])[0] for y in points["row"]])
    # A constant column displacement has no influence on the slope statistic.
    span = np.ptp(points["row"])
    observed, modeled = [np.polyfit(points["row"], x, 1)[0] * span
                         for x in (points["observed_x"], pred)]
    return observed, modeled, np.mean(abs(pred - points["observed_x"]))


if __name__ == "__main__":
    grid = range(-25, 26)
    def objective(angle):
        residuals = [optical(poses[i], angle) for i in (1, 2)]
        return np.mean([abs(a-b) for a,b,_ in residuals]), abs(angle)
    best = min(grid, key=objective)
    print("Optical fit:",best)
    fig, axes = plt.subplots(1,3, figsize=(12,4))
    with (HERE / "wrist-pitch.csv").open("w") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["trial","pitch_deg","observed_span_px","sim_span_px","edge_mae_px",
                         "push_error_mm","settled_error_mm","peak_tilt_deg"])
        for angle, marker in ((0,"o"), (best,"x")):
            rows = []
            for pose in poses:
                trial = pose["trial"]
                row = [trial, angle, *optical(pose,angle),
                       *score((trial,2), wrist_pitch_deg=angle)[2:]]
                writer.writerow(row)
                rows.append(row)
            data = np.array(rows)
            for ax, col in zip(axes,(3,5,6)):
                ax.scatter(data[:,0],data[:,col],marker=marker,label=f"{angle}°")
    axes[0].scatter(data[:,0],data[:,2],marker="_",color="black",label="Camera")
    for ax, title in zip(axes,("Edge shift across 75 rows (px)", "Pre-lift error (mm)", "Endpoint error (mm)")):
        ax.set(title=title, xlabel="Shelf trial", xticks=range(1,6))
        ax.legend()
    fig.tight_layout()
    fig.savefig(HERE / "wrist-pitch.png", dpi=160)

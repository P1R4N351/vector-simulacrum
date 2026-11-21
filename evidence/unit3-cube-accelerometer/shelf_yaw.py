"""Estimate in-plane cube rotation from the four published overhead glyph corners."""
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
rows = list(csv.DictReader((here / "shelf-slide.csv").open()))
corners = np.float32([[row[key].split() for key in row if key.startswith("glyph_")] for row in rows])
square = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
rectify = cv2.getPerspectiveTransform(corners[0], square)
axes = plt.subplots(figsize=(7, 4), layout="constrained")[1]
results = []
for row, quad in zip(rows, corners):
    points = cv2.perspectiveTransform(quad[None], rectify)[0]
    edges = np.roll(points, -1, axis=0) - points
    # Camera y increases downward. Report counterclockwise seen from above.
    angles = -np.angle((edges[:, 0] + 1j * edges[:, 1]) / np.array([1, 1j, -1, -1j]), deg=True)
    results.append([float(row["time_s"]), angles.mean(), angles.min(), angles.max()])
with (here / "shelf-yaw-camera.csv").open("w") as out:
    writer = csv.writer(out, lineterminator="\n")
    writer.writerow(["time_s", "mean_deg", "min_edge_deg", "max_edge_deg"])
    writer.writerows(np.round(results, 3))
t, mean, low, high = np.array(results).T
axes.errorbar(t, mean, yerr=[mean-low, high-mean], fmt="ko", capsize=3, label="Camera (four-edge range)")
sim = np.genfromtxt(here / "shelf-yaw-sim.csv", delimiter=",", names=True, dtype=None, encoding="utf8")
for name in dict.fromkeys(sim["initialization"]):
    data = sim[sim["initialization"] == name]
    axes.plot(data["time_s"], data["yaw_change_deg"], label=name)
axes.set(xlabel="Seconds after pickup command", ylabel="Yaw change (°, counterclockwise positive)", title="Cube rotation during the shelf push")
axes.grid(alpha=.25)
axes.legend(fontsize=8)
axes.figure.savefig(here / "shelf-yaw.png", dpi=155)
for time in [2.59, 2.86]:
    print(sim[np.isclose(sim["time_s"], time)])

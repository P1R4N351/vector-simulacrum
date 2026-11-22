"""Compare W13's optical cube pose with unit 3's retained cube occupancy."""
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PolyCollection
from matplotlib.path import Path as Polygon
from scipy.sparse.csgraph import connected_components

from simulacrum import dock, mjcf, params

here = Path(__file__).resolve().parent
data = json.loads((here.parents[2] / "web/public/robot/vector2/shelf-navmap.json").read_text())
survey = mjcf.shelf_survey()
outline = Polygon(survey["outline_mm"])
seated_xy, heading, _, _ = dock.docked_robot_pose(survey["dock_pose_mm"])
assert abs(heading) < 1e-8
with (here / "W13-registered.csv").open() as handle:
    cube = next(row for row in csv.DictReader(handle) if row["object"] == "cube")
optical = np.array([float(cube["x_mm"]), float(cube["y_mm"])])
corners = np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]])
def square(center, side, angle):
    c, s = np.cos(angle), np.sin(angle)
    return center + side * corners @ np.array([[c,s],[-s,c]])

patches, weights = [], []
for x, y, width, kind, angle in data["latest_objects_and_visibility"]:
    if kind == 3:
        patches.append(square(np.array(seated_xy) + [x,y], width, angle))
        weights.append(width**2)
centers = np.mean(patches, axis=1)
# Leaves share a quadtree orientation. Undo registration rotation before testing
# shared edges; the tolerance accounts for the export's 0.01 mm rounding.
angle = next(row[4] for row in data["latest_objects_and_visibility"] if row[3]==3)
c, s = np.cos(angle), np.sin(angle)
axes = centers @ np.array([[c,-s],[s,c]])
dist = np.abs(axes[:,None,:] - axes[None,:,:])
widths = np.sqrt(weights)
limit = (widths[:,None] + widths[None,:]) / 2
touch = (dist.max(axis=2) <= limit + .03) & (dist.min(axis=2) < limit - .03)
count, labels = connected_components(touch, directed=False)
centroids = np.array([np.average(centers[labels==k], weights=np.array(weights)[labels==k], axis=0)
                      for k in range(count)])
for k, center in enumerate(centroids):
    print("component",k,"leaves",sum(labels==k),"area mm2",sum(np.array(weights)[labels==k]),"center",center,
          "camera separation mm",np.linalg.norm(center-optical))
observed = np.array([np.array(seated_xy) + data["cell_mm"]*(np.array([i,j])+.5)
                     for i,j,k,_ in data["cells"] if k==2])
floor_corners = observed[:,None,:] + corners * data["cell_mm"]
side = params.values(params.load(model="vector_2"))["cube_size_mm"]
footprint = square(optical, side, float(cube["yaw_rad"]))



print("floor cell centres inside",sum(outline.contains_points(observed)),"of",len(observed))
print("floor cells with all corners inside",outline.contains_points(floor_corners.reshape(-1,2)).reshape(-1,4).all(axis=1).sum())
fig, ax = plt.subplots(figsize=(5.2,4.8), layout="constrained")
ax.add_collection(PolyCollection(patches, facecolor="#d3dbea", edgecolor="#8c9db8", linewidth=.45))
ax.plot(*np.vstack([footprint,footprint[0]]).T, color="#086b64", label="W13 camera footprint")
ax.scatter(*optical, color="#086b64", marker="+", s=110)
ax.scatter(*centroids.T, color="#552f71", marker="x", label="Component centroids")
ax.set(aspect="equal",xlabel="Outward from dock (mm)",ylabel="Left (mm)",title="W13 cube sighting and later native map")
ax.autoscale(); ax.legend(loc="upper left",bbox_to_anchor=(0,-.18),ncol=2,fontsize=8)
fig.savefig(here/"cube-map-check.png", dpi=180)

"""Compare floor support in the cube body frame. Supply deployed OBJ and scene XML."""
import argparse
import csv
import itertools
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("obj", type=Path)
parser.add_argument("scene", type=Path)
args = parser.parse_args()
vertices = []
used = set()
for line in args.obj.read_text().splitlines():
    if line.startswith("v "):
        vertices.append(list(map(float, line.split()[1:4])))
    elif line.startswith("f "):
        used.update(int(word.split("/")[0]) - 1 for word in line.split()[1:])
assert min(used) >= 0 and max(used) < len(vertices)
surface = np.array(vertices)[sorted(used)] * 1000
body = ET.parse(args.scene).find('.//body[@name="cube"]')
geoms = [geom for geom in body.findall("geom") if geom.get("contype") != "0"]
assert all(g.get("type") == "box" and set(g.attrib).isdisjoint(
    {"quat","axisangle","xyaxes","zaxis","euler","fromto"}) for g in geoms)
centers, sizes = [np.array([np.fromstring(g.get(key), sep=" ") for g in geoms]) * 1000
                  for key in ("pos","size")]
angles = np.arange(91)
rad = np.deg2rad(angles)
fig, ax = plt.subplots(figsize=(7.7, 4.4), layout="constrained")
with open("support.csv", "w", newline="") as out:
    writer = csv.writer(out, lineterminator="\n")
    writer.writerow(["plane","sign_a","sign_b","deg","contact_mm","optical_mm","gap_mm"])
    for a,b in itertools.combinations(range(3),2):
        plane = "XYZ"[a] + "XYZ"[b]
        gaps = []
        for sa,sb in itertools.product((-1,1), repeat=2):
            normals = np.zeros((3,len(rad)))
            normals[a] = sa*np.cos(rad)
            normals[b] = sb*np.sin(rad)
            contact = (centers @ normals + sizes @ abs(normals)).max(axis=0)
            optical = (surface @ normals).max(axis=0)
            gap = contact-optical
            gaps.append(gap)
            writer.writerows(zip([plane]*91,[sa]*91,[sb]*91,angles,contact,optical,gap))
        ax.plot(angles,np.mean(gaps,axis=0),label=plane)
        ax.fill_between(angles,np.min(gaps,axis=0),np.max(gaps,axis=0),alpha=.15)
ax.axhline(0,color=".45",lw=1)
ax.set(xlabel="Rotation between adjacent face normals (degrees)", ylabel="Visible clearance at rigid contact (mm)",
       title="Cube corner clearance during an edge roll")
ax.legend(title="Body plane · mean over signs")
fig.savefig("support.png",dpi=180)

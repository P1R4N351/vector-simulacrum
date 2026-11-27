"""Suppress one side of the upper fork above a wrist-relative cut plane."""
import csv
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull


def trim(source, target, side, cut_mm):
    tree = ET.parse(source)
    fork = tree.find(".//body[@name='fork']")
    assets = tree.find("asset")
    for g in list(fork.findall("geom[@mesh]")):
        mesh = assets.find(f"mesh[@name='{g.get('mesh')}']")
        p = np.fromstring(mesh.get("vertex"), sep=" ").reshape(-1, 3) * 1000
        if p[:, 2].max() <= cut_mm or (side * p[:, 1]).max() <= 0:
            continue
        # No piece spanning the fork midline should be on the cutting plane.
        assert (side * p[:, 1]).min() > 0
        if p[:, 2].min() >= cut_mm:
            fork.remove(g)
            assets.remove(mesh)
            continue
        pts = list(p[p[:, 2] <= cut_mm])
        for a, b, c in np.fromstring(mesh.get("face"), sep=" ", dtype=int).reshape(-1, 3):
            for u, v in ((a,b), (b,c), (c,a)):
                if (p[u,2]-cut_mm)*(p[v,2]-cut_mm) < 0:
                    pts.append(p[u]+(p[v]-p[u])*(cut_mm-p[u,2])/(p[v,2]-p[u,2]))
        pts = np.unique(np.round(pts, 9), axis=0)
        verts = pts[ConvexHull(pts).vertices] * .001
        mesh.set("vertex", " ".join(f"{x:.12g}" for x in verts.ravel()))
        del mesh.attrib["face"]  # MuJoCo constructs the convex hull at compile time.
    tree.write(target)


def main():
    import evaluate
    source = Path(sys.argv[2]).resolve()
    rows = []
    with TemporaryDirectory() as tmp:
        for side, cut in [(0,0), (1,-8), (-1,-8), (1,-16), (-1,-16)]:
            if side:
                fixture = Path(tmp) / "fork.xml"
                trim(source, fixture, side, cut)
            else:
                fixture = source
            sys.argv[2] = str(fixture)
            for trial in range(1,6):
                score, trace = evaluate.run((trial, "stl", "overhead", "weld", 0, 0, 0))
                a = np.asarray(trace)
                right = np.interp([5.498, 5.998], a[:,0]-1.5, -a[:,2])
                rows.append((trial, side, cut, *right, -score[9], score[14], score[12]))
    headers = ["trial","side_sign","wrist_cut_z_mm","right_5_498_mm","right_5_998_mm",
               "final_right_mm","peak_tilt_deg","endpoint_error_mm"]
    out = Path(__file__).resolve().parent
    with (out/"unilateral-fork.csv").open("w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(headers)
        writer.writerows(rows)
    fig, panels = plt.subplots(1, 2, figsize=(9, 3.8), layout="constrained")
    for group in range(5):
        subset = np.array(rows[group*5:group*5+5])
        side, cut = subset[0,1:3]
        label = "Intact" if side == 0 else f"{'Left' if side > 0 else 'Right'} cut at {cut:g} mm"
        panels[0].plot(range(3), subset[0,3:6], "o-", label=label)
        panels[1].plot(range(2,6), subset[1:,-1], "o-")
    with (out/"optical-clearance.csv").open() as stream:
        poses = [r for r in csv.DictReader(stream) if r["side"] == "right"]
    for index, exposure in enumerate((5.5, 6.0)):
        right = [-float(r["left_mm"]) for r in poses if float(r["replay_sec"]) == exposure]
        panels[0].vlines(index, min(right), max(right), color="black", linewidth=4,
                         label="Optical solutions" if index == 0 else None)
    panels[0].scatter([2], [-evaluate.observed["over_left"][0]], marker="*", color="black")
    panels[0].set(xticks=range(3), xticklabels=["5.50 s","6.00 s","Settled"],ylabel="Rightward travel (mm)",title="Sideways sweep")
    panels[1].set(xticks=range(2,6), xlabel="Push trial", ylabel="Endpoint vector error (mm)",title="Other pushes")
    panels[0].legend(fontsize=7)
    fig.savefig(out/"unilateral-fork.png",dpi=160)


if __name__ == "__main__":
    main()

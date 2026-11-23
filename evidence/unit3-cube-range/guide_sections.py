"""Ray-section the pinned charger OBJ and the simulator's side guides at tread heights."""
import csv
import hashlib
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulacrum.dock import load_surface_boxes, obj_to_charger_mm


def mesh_triangles():
    source = ROOT/"web/public/robot/sdk/Vic_charger_hp_geo.obj"
    assert hashlib.sha256(source.read_bytes()).hexdigest() == "2ccfb20d5ed4b793862f8deea940632a2f4f34a0dfc76f8fd6083e53419b3bb0"
    vertices, faces = [], []
    for line in source.read_text().splitlines():
        if line.startswith("v "):
            vertices.append([float(v) for v in line.split()[1:4]])
        elif line.startswith("f "):
            indices = [int(v.split("/")[0])-1 for v in line.split()[1:]]
            faces.extend((indices[0], a, b) for a, b in zip(indices[1:-1], indices[2:]))
    return obj_to_charger_mm(np.asarray(vertices))[np.asarray(faces)]


def ray_hits(triangles, x, z):
    origin = triangles[:, 0]
    a, b = triangles[:, 1] - origin, triangles[:, 2] - origin
    det = a[:, 0]*b[:, 2] - a[:, 2]*b[:, 0]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = ((x-origin[:, 0])*b[:, 2] - (z-origin[:, 2])*b[:, 0])/det
        v = (a[:, 0]*(z-origin[:, 2]) - a[:, 2]*(x-origin[:, 0]))/det
        hit = (abs(det)>1e-9) & (u>=-1e-9) & (v>=-1e-9) & (u+v<=1+1e-9)
    return origin[hit, 1] + u[hit]*a[hit, 1] + v[hit]*b[hit, 1]


def guide_triangles():
    triangles = []
    for box in load_surface_boxes():
        if box.part.split("_")[0] not in {"funnel", "rail"}:
            continue
        qw, _, _, qz = box.quat
        co, si = qw*qw-qz*qz, 2*qw*qz
        vertices = np.array([[i,j,k] for i in [-1,1] for j in [-1,1] for k in [-1,1]]) * box.half_mm
        vertices = vertices @ np.array([[co,si,0],[-si,co,0],[0,0,1]]) + box.center_mm
        for face in [[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]]:
            triangles.extend(vertices[[face[0],face[i],face[i+1]]] for i in [1,2])
    return np.asarray(triangles)


def main():
    mesh, boxes = mesh_triangles(), guide_triangles()
    fig, axes = plt.subplots(2, 3, figsize=(12, 6), sharex=True, sharey=True, layout="constrained")
    rows = []
    for ax,z in zip(axes.flat, [10,12,14,16,18,20], strict=True):
        section = []
        for x in np.arange(24.25, 48, .5):
            row = [x,z]
            for tris in [mesh, boxes]:
                hits = ray_hits(tris, x, z)
                for sign in [1,-1]:
                    side = sign*hits[sign*hits>0]
                    row.append(min(side) if len(side) else np.nan)
            rows.append(row)
            section.append(row)
        x, _, left, right, box_left, box_right = np.array(section).T
        ax.plot(x,left,label="OBJ +y")
        ax.plot(x,right,"--",label="OBJ −y")
        ax.plot(x,box_left,label="Guide box +y")
        ax.plot(x,box_right,":",label="Guide box −y")
        ax.set_title(f"{z} mm above dock datum")
    for ax in axes[-1]:
        ax.set_xlabel("Inward from dock lip (mm)")
    for ax in axes[:,0]:
        ax.set_ylabel("Centerline to first surface (mm)")
    axes[0,0].legend(fontsize=8)
    fig.savefig(Path(__file__).with_name("guide-sections.png"),dpi=160)
    with Path(__file__).with_name("guide-sections.csv").open("w") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow("x_mm z_mm obj_pos_y_mm obj_neg_y_abs_mm box_pos_y_mm box_neg_y_abs_mm".split())
        writer.writerows(rows)


if __name__ == "__main__":
    main()

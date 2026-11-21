"""Compare the forward envelope of the registered STL and its convex collision union."""
import csv
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[1]))
from simulacrum import params

STEP = .1


def forward_map(triangles, y, z):
    depth = np.full((len(z), len(y)), -np.inf)
    for tri in triangles:
        a, b, c = tri
        u, v = b-a, c-a
        det = u[1]*v[2]-v[1]*u[2]
        if abs(det) < 1e-10:
            continue
        lower, upper = tri.min(axis=0), tri.max(axis=0)
        yi, yj = y.searchsorted(lower[1]), y.searchsorted(upper[1], side="right")
        zi, zj = z.searchsorted(lower[2]), z.searchsorted(upper[2], side="right")
        dy, dz = y[None, yi:yj]-a[1], z[zi:zj, None]-a[2]
        p, q = (dy*v[2]-dz*v[1])/det, (dz*u[1]-dy*u[2])/det
        inside = (p>=0) & (q>=0) & (p+q<=1)
        region = depth[zi:zj, yi:yj]
        np.maximum(region, a[0]+p*u[0]+q*v[0], out=region, where=inside)
    return depth


def registered_fork(stl):
    record = np.dtype([("normal","<f4",3), ("tri","<f4",(3,3)), ("attribute","<u2")])
    xyz = np.frombuffer(stl.read_bytes(), dtype=record, offset=84)["tri"].astype(float)
    x,y,z = xyz.transpose(2,0,1)
    v = params.values(params.load(model="vector_2"))
    return np.stack((-z+v["lift_front_wrt_wrist_mm"]-26.52, -x-48.01,
                       y+v["lift_xbar_top_wrt_wrist_mm"]-v["lift_xbar_height_mm"]+36.79), axis=-1)


def main(stl, fixture):
    source = registered_fork(stl)
    tree = ET.parse(fixture)
    meshes = {m.get("name"):m for m in tree.findall(".//asset/mesh")}
    pieces = []
    for g in tree.findall(".//body[@name='fork']/geom[@type='mesh']"):
        mesh = meshes[g.get("mesh")]
        vertices = np.fromstring(mesh.get("vertex"),sep=" ").reshape(-1,3)*1000
        faces = np.fromstring(mesh.get("face"),sep=" ",dtype=int).reshape(-1,3)
        pieces.append(vertices[faces])
    hulls = np.concatenate(pieces)
    points = np.concatenate((source,hulls)).reshape(-1,3)
    y,z = [np.arange(np.floor(low)+STEP/2,np.ceil(high),STEP)
           for low,high in zip(points.min(axis=0)[1:],points.max(axis=0)[1:])]
    original, collision = [forward_map(tri,y,z) for tri in (source,hulls)]
    common = np.isfinite(original) & np.isfinite(collision)
    delta = np.full_like(original,np.nan)
    np.subtract(collision,original,out=delta,where=common)
    masks = {"both":common, "hull_only":np.isfinite(collision)&~common,
             "stl_only":np.isfinite(original)&~common, "over_1_mm":delta>1, "under_minus_1_mm":delta<-1}
    with (here/"fork-envelope.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow(["region","projected_area_mm2"])
        for name,mask in masks.items():
            writer.writerow([name, round(mask.sum()*STEP**2,3)])
    print("forward x/y/z min max",source.min(axis=(0,1)),source.max(axis=(0,1)))
    print("absolute depth difference percentiles",np.percentile(abs(delta[common]),[50,90,95,99,100]))
    extent=[y[0]-STEP/2,y[-1]+STEP/2,z[0]-STEP/2,z[-1]+STEP/2]
    fig,(ax,bx)=plt.subplots(1,2,figsize=(10,4.2),layout="constrained",sharey=True)
    for panel,im,title in [(ax,np.where(np.isfinite(original),original,np.nan),"STL leading surface"),(bx,delta,"Collision surface minus STL")]:
        opts=dict(cmap="coolwarm",vmin=-1,vmax=1) if panel==bx else dict(cmap="viridis")
        surface=panel.imshow(im,origin="lower",extent=extent,**opts)
        fig.colorbar(surface,ax=panel,label="Forward depth (mm)")
        panel.set(title=title,xlabel="Distance left of wrist (mm)")
    ax.set_ylabel("Height above wrist (mm)")
    fig.savefig(here/"fork-envelope.png",dpi=160)
    plt.close(fig)


if __name__=="__main__":
    main(Path(sys.argv[1]),Path(sys.argv[2]))

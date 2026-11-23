"""Bracket an unknown fracture by trimming one lower contact lip in wrist coordinates."""
import argparse
import csv
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from scipy.spatial import ConvexHull

from wrist_signal import compare, params, replay, root


def half(vertices, axis, plane, sign):
    """Vertices of a convex piece on one side, including new edge intersections."""
    dist = (vertices[:, axis] - plane) * sign
    points = list(vertices[dist >= 0])
    for tri in ConvexHull(vertices).simplices:
        for a, b in zip(tri, np.roll(tri, 1)):
            if dist[a]*dist[b] < 0:
                points.append(vertices[a] + dist[a]/(dist[a]-dist[b])*(vertices[b]-vertices[a]))
    points = np.unique(points, axis=0)
    if len(points) < 4 or np.linalg.matrix_rank(points - points[0], tol=1e-10) < 3:
        return None
    return points[ConvexHull(points).vertices]


def trim(path, output, side, xlimit):
    tree = ET.parse(path)
    assets = tree.find('asset')
    fork = tree.find(".//body[@name='fork']")
    removed = 0.
    for geom in list(fork.findall("geom[@mesh]")):
        mesh = assets.find("mesh[@name='%s']" % geom.get('mesh'))
        points = np.fromstring(mesh.get('vertex'),sep=' ').reshape(-1,3)
        # Remove only material on the chosen side, below the wrist mounts and
        # ahead of the candidate break plane. Split crossing convex pieces.
        keep = []
        cut = points
        for axis, plane, sign in [(1,0,side),(2,-.012,-1),(0,xlimit,1)]:
            dist = (cut[:,axis]-plane)*sign
            if dist.max() <= 0:
                cut = None
                break
            if dist.min() < 0:
                part = half(cut, axis, plane, -sign)
                if part is not None:
                    keep.append(part)
                cut = half(cut, axis, plane, sign)
                if cut is None:
                    break
        if cut is None:
            continue
        removed += ConvexHull(cut).volume
        assets.remove(mesh)
        fork.remove(geom)
        for i, part in enumerate(keep):
            name = mesh.get('name') + '_clip' + str(i)
            ET.SubElement(assets,'mesh',name=name,vertex=' '.join(f'{v:.10g}' for v in part.flat))
            ET.SubElement(fork,'geom', dict(geom.attrib,mesh=name,name=name))
    assert removed > 0
    tree.write(output)
    return removed*1e9


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('fixture')
    args = p.parse_args()
    clock = np.arange(-.35,2.45,.001)
    def channel(file):
        data = np.loadtxt(root/file,delimiter=',',skiprows=1)
        return np.interp(clock,data[:,0],data[:,1])
    v = params.values(params.load(model='vector_2'))
    heights, pitch = channel('encoder.csv'), channel('dock-pitch.csv')
    fig, (ax, patch) = plt.subplots(1, 2, figsize=(10,4), layout='constrained')
    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        for side,limit in [(0,0),(1,3),(-1,3),(1,1),(-1,1)]:
            path = args.fixture
            volume = 0
            label = 'Intact'
            if side:
                path = Path(tmp)/'trim.xml'
                volume = trim(args.fixture,path,side,limit/1000)
                label = ('Left' if side==1 else 'Right') + f' · X limit {limit} mm'
            loaded_z = []
            def observe(t, model, d, fork, cube):
                if t < 0:
                    return
                for i,c in enumerate(d.contact):
                    if set(model.geom_bodyid[c.geom]) != {fork,cube} or c.efc_address < 0:
                        continue
                    force = np.zeros(6)
                    mj.mj_contactForce(model,d,i,force)
                    if force[0] > 1e-5:
                        local = d.xmat[fork].reshape(3,3).T @ (c.pos-d.xpos[fork])
                        loaded_z.append(local[2]*1000)
            packets = replay(path, True, v, clock, heights, pitch, observe)
            trace = compare([(t,p) for t,p in packets if -.21<=t<=2.45])
            t,score = trace[trace[:,0]>=0,:2].T
            rows.append([label,volume,score.max(),t[score.argmax()],score[t>=1.35].max(), min(loaded_z,default=None), max(loaded_z,default=None)])
            if side == 0:
                ax.plot(t,score,label='Intact and four trimmed variants')
            index = len(rows)-1
            patch.plot([index,index],[min(loaded_z),max(loaded_z)],lw=4,color='tab:blue')
    ax.set(xlabel='Seconds from lift command',ylabel='Movement listener score',
           title='Measured wrist and pitch · centered first touch')
    ax.legend(fontsize=8)
    patch.axhspan(-30,-12,color='tab:orange',alpha=.25,label='Trimmed height range')
    patch.set(xticks=range(5),xticklabels=['Intact','L3','R3','L1','R1'],
              ylabel='Height relative to wrist (mm)',title='Loaded contact height range')
    patch.legend(fontsize=8)
    fig.savefig(root/'fork-damage.png',dpi=160)
    with (root/'fork-damage.csv').open('w') as f:
        w=csv.writer(f)
        w.writerow(['variant','removed_convex_volume_sum_mm3','peak','peak_packet_s','peak_after_1.35s','min_loaded_contact_z_mm','max_loaded_contact_z_mm'])
        w.writerows(rows)


if __name__ == '__main__':
    main()

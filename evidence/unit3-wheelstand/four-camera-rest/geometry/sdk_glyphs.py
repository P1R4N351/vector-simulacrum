"""Measure the browser's glyph borders through the pinned SDK texture coordinates."""
import csv
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ASSETS = HERE.parents[3] / "web/public/robot/sdk"


def read_mesh():
    vertices, uv, triangles = [], [], []
    for line in (ASSETS/"cube.obj").read_text().splitlines():
        parts = line.split()
        if line.startswith("v "):
            vertices.append(list(map(float, parts[1:])))
        if line.startswith("vt "):
            uv.append(list(map(float, parts[1:3])))
        if line.startswith("f "):
            indices = [tuple(int(k)-1 for k in p.split("/")[:2]) for p in parts[1:]]
            triangles.extend([[indices[0],a,b] for a,b in zip(indices[1:],indices[2:])])
    vertices = np.asarray(vertices)
    vertices = (vertices - (vertices.max(0)+vertices.min(0))/2)*44/np.ptp(vertices,axis=0)
    indices = np.asarray(triangles)
    return vertices[indices[:,:,0]], np.asarray(uv)[indices[:,:,1]]


def position(point, tri, uv):
    hits = []
    for xyz, texture in zip(tri,uv):
        matrix = np.vstack([texture.T,np.ones(3)])
        if abs(np.linalg.det(matrix)) < 1e-12:
            continue
        weights = np.linalg.solve(matrix,[*point,1])
        hits.append((min(weights), weights @ xyz))
    margin, xyz = max(hits, key=lambda hit: hit[0])
    assert margin > -.12
    return xyz, margin


def main():
    tri,uv = read_mesh()
    atlas = cv2.imread(str(ASSETS/"cube.jpg"),cv2.IMREAD_GRAYSCALE)
    h,w=atlas.shape
    contours,_=cv2.findContours(np.uint8(atlas[:370,:260]<128),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    rects=sorted([cv2.boundingRect(c) for c in contours if cv2.contourArea(c)>9000],key=lambda r:(r[1]//10,r[0]))
    assert len(rects)==6
    with (HERE/"sdk-glyphs.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow(["atlas_row","atlas_col","sdk_face","horizontal_mm","vertical_mm","recess_mm","center_offset_mm","all_midpoints_on_mesh"])
        for i,(x,y,bw,bh) in enumerate(rects):
            # Pixel edges, normalized to the same top-origin atlas used by stockCubeTexture.
            points=np.array([[x,y+bh/2],[x+bw,y+bh/2],[x+bw/2,y],[x+bw/2,y+bh]])/[w,h]
            sample=[position(p,tri,uv) for p in points]
            corners=np.array([xyz for xyz,_ in sample])
            normal=int(np.argmin(np.ptp(corners,axis=0)))
            lengths=np.linalg.norm(corners[1::2]-corners[::2],axis=1)
            center=corners.mean(axis=0)
            writer.writerow([i//2,i%2,"XYZ"[normal]+("+" if center[normal]>0 else "-"),
                             *lengths,22-abs(center[normal]),np.linalg.norm(np.delete(center,normal)),min(m for _,m in sample)>=-1e-7])


if __name__=="__main__":
    main()

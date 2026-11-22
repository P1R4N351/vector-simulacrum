"""Distance checks on cube and fork source triangles, in millimetres."""
import numpy as np


def outer_face(points,axis,tri):
    """Outermost triangle intersection on the lateral axis at each contact."""
    uv=[i for i in range(3) if i!=axis]
    extent=np.full(len(points),-np.inf)
    for a,b,c in tri:
        matrix=np.column_stack(((b-a)[uv],(c-a)[uv]))
        if abs(np.linalg.det(matrix)) < 1e-10:
            continue
        weights=(points[:,uv]-a[uv])@np.linalg.inv(matrix).T
        mask=(weights.min(axis=1)>=-1e-9)&(weights.sum(axis=1)<=1+1e-9)
        coord=a[axis]+weights[mask]@np.array([b[axis]-a[axis],c[axis]-a[axis]])
        extent[mask]=np.maximum(extent[mask],coord*(1 if axis else -1))
    return extent

def nearest_distance(points, triangles):
    """Minimum over face interiors and finite edges, including degenerate facets."""
    best = np.full(len(points), np.inf)
    radius = 3.0
    nearby = ((triangles.max(axis=1) >= points.min(axis=0)-radius) &
              (triangles.min(axis=1) <= points.max(axis=0)+radius)).all(axis=1)
    for a,b,c in triangles[nearby]:
        for start,end in ((a,b),(b,c),(c,a)):
            edge = end-start
            length2 = edge@edge
            if length2:
                rel = points-start
                u = np.clip((rel@edge)/length2,0,1)
                best = np.minimum(best,np.linalg.norm(rel-u[:,None]*edge,axis=1))
        ab,ac = b-a,c-a
        normal = np.cross(ab,ac)
        norm = np.linalg.norm(normal)
        if norm < 1e-10:
            continue
        normal /= norm
        inside = np.ones(len(points),dtype=bool)
        for start,end in ((a,b),(b,c),(c,a)):
            inside &= (points-start)@np.cross(normal,end-start) >= -1e-12
        best[inside] = np.minimum(best[inside],abs((points[inside]-a)@normal))
    assert best.max() < radius  # no omitted facet can be closer
    return best

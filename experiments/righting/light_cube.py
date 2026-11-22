"""Approximate the SDK cube's concave surface with nonoverlapping contact boxes."""

import csv
import itertools
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

try:
    from corners import split_caps
except ImportError:
    from .corners import split_caps


def sdk_surface(path: Path):
    """Load an OBJ mesh and normalize it to the engine's 44 mm cube envelope."""
    assert path.is_file(), f"missing cube mesh: {path}"
    vertices, triangles = [], []
    for line in path.read_text().splitlines():
        fields = line.split()
        if fields[:1] == ["v"]:
            vertices.append([float(x) for x in fields[1:4]])
        elif fields[:1] == ["f"]:
            indices = [int(word.split("/")[0]) - 1 for word in fields[1:]]
            if min(indices) < 0:
                raise ValueError("expected positive OBJ indices")
            triangles.extend(
                (indices[0], i, j) for i, j in zip(indices[1:], indices[2:])
            )
    assert vertices and triangles, "cube mesh has no geometry"
    verts = np.array(vertices, dtype=float)
    # Normalize the visual mesh to the engine's 44 mm envelope on each axis.
    verts = (verts - (verts.max(0) + verts.min(0)) / 2) * 0.044 / np.ptp(verts, axis=0)
    return verts, verts[triangles]


def planes(coords, tol: float = 0.00025):
    """Cluster 1D coordinates separated by at most `tol` metres into cell boundaries."""
    assert len(coords) > 0 and tol > 0.0, "empty coordinate array or non-positive tolerance"
    groups = []
    for x in np.unique(coords):
        if not groups or x - groups[-1][0] > tol:
            groups.append([])
        groups[-1].append(x)
    assert len(groups) >= 2, "expected distinct coordinate planes"
    result = np.array([np.mean(g) for g in groups])
    result[0], result[-1] = min(coords), max(coords)
    return result


def symmetric_xy_planes(x_coords, y_coords, tol: float = 0.00025):
    """Cluster pooled |x| and |y| planes so all four lateral faces share identical D4 boundaries."""
    assert len(x_coords) > 0 and len(y_coords) > 0, "empty x or y coordinates"
    assert tol > 0.0, "plane tolerance must be positive"
    pooled = np.sort(np.unique(np.abs(np.concatenate([x_coords, y_coords]))))
    groups = []
    for val in pooled:
        if not groups or val - groups[-1][0] > tol:
            groups.append([])
        groups[-1].append(val)
    pos = np.array([np.mean(g) for g in groups])
    pos = pos[pos > 1e-6]
    assert len(pos) >= 1, "no non-zero XY planes"
    pos[-1] = 0.022
    return np.concatenate([-pos[::-1], pos])


def inside(points, triangles):
    """Return boolean occupancy for query points inside a closed triangle mesh."""
    assert len(points) > 0 and len(triangles) > 0, "empty query points or triangles"
    assert points.ndim == 2 and points.shape[1] == 3, "query points must be Nx3"
    # Slightly oblique rays avoid counting shared triangle edges twice.
    direction = np.array([1.0, 0.000178, 0.000257])
    counts = np.zeros(len(points), dtype=int)
    for a, b, c in triangles:
        normal = np.cross(direction, c - a)
        det = np.dot(b - a, normal)
        if abs(det) < 1e-12:
            continue
        delta = points - a
        u = delta @ normal / det
        cross = np.cross(delta, b - a)
        v = cross @ direction / det
        distance = cross @ (c - a) / det
        counts += (u >= 0) & (v >= 0) & (u + v <= 1) & (distance > 0)
    return counts % 2 == 1


def _symmetrize_d4(occupied: np.ndarray) -> np.ndarray:
    """Enforce exact 8-fold D4 dihedral symmetry in XY across the voxel grid."""
    assert occupied.ndim == 3 and occupied.shape[0] == occupied.shape[1], "XY grid must be square"
    transposed = np.transpose(occupied, (1, 0, 2))
    votes = (
        occupied.astype(int)
        + occupied[::-1, :, :]
        + occupied[:, ::-1, :]
        + occupied[::-1, ::-1, :]
        + transposed
        + transposed[::-1, :, :]
        + transposed[:, ::-1, :]
        + transposed[::-1, ::-1, :]
    )
    sym = votes >= 4
    assert np.array_equal(sym, np.transpose(sym[::-1, :, :], (1, 0, 2))), "D4 rotation invariant failed"
    return sym


def _merge_boxes(occupied: np.ndarray, edges):
    """Greedily merge adjacent occupied cells into nonoverlapping axis-aligned boxes."""
    assert len(edges) == 3 and occupied.ndim == 3, "expected 3D cell grid"
    assert all(len(e) == s + 1 for e, s in zip(edges, occupied.shape)), "edges must bracket occupied grid"
    shape = occupied.shape
    boxes = []
    for origin in np.ndindex(shape):
        if not occupied[origin]:
            continue
        end = np.array(origin) + 1
        for axis in range(3):
            while end[axis] < shape[axis]:
                slab = [slice(a, b) for a, b in zip(origin, end)]
                slab[axis] = end[axis]
                if not occupied[tuple(slab)].all():
                    break
                end[axis] += 1
        occupied[tuple(slice(a, b) for a, b in zip(origin, end))] = False
        lo = np.array([e[i] for e, i in zip(edges, origin)])
        hi = np.array([e[i] for e, i in zip(edges, end)])
        boxes.append([((hi + lo) / 2).tolist(), ((hi - lo) / 2).tolist()])
    return boxes


def collision_boxes(path: Path):
    """Decompose `cube.obj` into nonoverlapping D4-symmetric contact boxes in metres."""
    verts, tris = sdk_surface(path)
    assert len(verts) > 0 and len(tris) > 0, "empty cube mesh"
    xy_edges = symmetric_xy_planes(verts[:, 0], verts[:, 1], tol=0.00025)
    z_edges = planes(verts[:, 2], tol=0.00025)
    edges = [xy_edges, xy_edges, z_edges]
    centers = [(axis[1:] + axis[:-1]) / 2 for axis in edges]
    shape = tuple(map(len, centers))
    points = np.array(list(itertools.product(*centers)))
    occupied = _symmetrize_d4(inside(points, tris).reshape(shape))
    if not occupied[tuple(n // 2 for n in shape)] or occupied.all():
        raise ValueError("cube asset lacks the expected solid core and recessed edges")
    return _merge_boxes(occupied, edges)


def add_cube(world, boxes, mass_g, friction, corner_friction=0.9):
    """Append a rigid-body cube with split corner-rubber contact boxes to MJCF `world`."""
    assert mass_g > 0.0 and friction > 0.0, "mass and friction must be positive"
    assert len(boxes) > 0, "no collision boxes supplied"
    cube = ET.SubElement(world, "body", name="cube")
    ET.SubElement(cube, "freejoint", name="cube_root")
    kg = mass_g * 0.001
    # Uniform-cube reference inertia; battery mass distribution has not been measured.
    ET.SubElement(
        cube,
        "inertial",
        pos="0 0 0",
        mass=str(kg),
        diaginertia=" ".join([str(kg * 0.044**2 / 6)] * 3),
    )
    parts = split_caps(boxes) if corner_friction is not None else (
        (pos, half, "shell") for pos, half in boxes
    )
    for index, (pos, size, material) in enumerate(parts):
        rubber = material == "rubber"
        ET.SubElement(
            cube,
            "geom",
            type="box",
            name=f"cube_{material}_{index}",
            pos=" ".join(map(str, pos)),
            size=" ".join(map(str, size)),
            contype="4",
            conaffinity="3",
            priority="2" if rubber else "1",
            friction=f"{corner_friction if rubber else friction} .005 .0001",
            **({"solref": ".008 1", "rgba": ".035 .035 .035 1"} if rubber else {}),
        )


def write_surface_csv(cube_obj: Path, csv_out: Path) -> int:
    """Regenerate `simulacrum/cube_surface.csv` in millimetres from `cube.obj`."""
    parts = list(split_caps(collision_boxes(cube_obj)))
    assert len(parts) > 0, "split_caps produced no boxes"
    rubber = sum(1 for _, _, mat in parts if mat == "rubber")
    assert rubber == 8, f"expected 8 rubber corner caps, got {rubber}"
    with csv_out.open("w", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(["material", "x_mm", "y_mm", "z_mm", "half_x_mm", "half_y_mm", "half_z_mm"])
        for pos, half, material in parts:
            vals = [f"{v * 1000.0:.9g}" for v in (*pos, *half)]
            writer.writerow([material, *vals])
    return len(parts)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: python3 light_cube.py <cube.obj> <cube_surface.csv>")
    count = write_surface_csv(Path(sys.argv[1]), Path(sys.argv[2]))
    print(f"wrote {count} boxes to {sys.argv[2]}")

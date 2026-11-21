"""Approximate the SDK cube's concave surface with nonoverlapping contact boxes."""

import itertools
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from corners import split_caps


def sdk_surface(path: Path):
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
    verts = np.array(vertices)
    # Normalize the visual mesh to the engine's 44 mm envelope on each axis.
    verts = (verts - (verts.max(0) + verts.min(0)) / 2) * 0.044 / np.ptp(verts, axis=0)
    return verts, verts[triangles]


def planes(coords):
    groups = []
    for x in np.unique(coords):
        if not groups or x - groups[-1][0] > 0.00065:
            groups.append([])
        groups[-1].append(x)
    result = np.array([np.mean(g) for g in groups])
    result[0], result[-1] = min(coords), max(coords)
    return result


def inside(points, triangles):
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


def collision_boxes(path):
    verts, tris = sdk_surface(path)
    edges = [planes(axis) for axis in verts.T]
    centers = [(axis[1:] + axis[:-1]) / 2 for axis in edges]
    shape = tuple(map(len, centers))
    points = np.array(list(itertools.product(*centers)))
    occupied = inside(points, tris).reshape(shape)
    if not occupied[tuple(n // 2 for n in shape)] or occupied.all():
        raise ValueError("cube asset lacks the expected solid core and recessed edges")
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


def add_cube(world, boxes, mass_g, friction, corner_friction=0.9):
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

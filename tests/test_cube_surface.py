"""Geometric, topological, and D4-symmetry invariants for simulacrum/cube_surface.csv."""

from __future__ import annotations

import csv
import importlib.util
from pathlib import Path
import sys
from typing import List, Tuple
import unittest

import numpy as np

import _common

CSV_PATH = _common.ROOT / "simulacrum" / "cube_surface.csv"
SDK_CUBE_PATH = _common.ROOT / "web" / "public" / "robot" / "sdk" / "cube.obj"
TOL_MM = 1e-4


def load_boxes(path: Path = CSV_PATH) -> List[Tuple[str, np.ndarray, np.ndarray]]:
    """Load `(material, center_mm, half_mm)` records from a cube surface CSV."""
    assert path.is_file(), f"missing cube surface file: {path}"
    rows: List[Tuple[str, np.ndarray, np.ndarray]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            center = np.array([float(row["x_mm"]), float(row["y_mm"]), float(row["z_mm"])])
            half = np.array([float(row["half_x_mm"]), float(row["half_y_mm"]), float(row["half_z_mm"])])
            rows.append((row["material"], center, half))
    assert len(rows) > 0, f"no boxes loaded from {path}"
    return rows


def max_pairwise_overlap_mm(rows: List[Tuple[str, np.ndarray, np.ndarray]]) -> float:
    """Return the maximum 3D intersection depth in millimetres across any pair of boxes."""
    assert len(rows) >= 2, "need at least two boxes to check overlap"
    worst = 0.0
    for i in range(len(rows)):
        _, ci, hi = rows[i]
        assert np.all(hi > 0.0), f"non-positive half extent at row {i}"
        for j in range(i + 1, len(rows)):
            _, cj, hj = rows[j]
            overlap = np.minimum(ci + hi, cj + hj) - np.maximum(ci - hi, cj - hj)
            if np.all(overlap > 0.0):
                worst = max(worst, float(np.min(overlap)))
    return worst


def union_mask_at(points_mm: np.ndarray, rows: List[Tuple[str, np.ndarray, np.ndarray]]) -> np.ndarray:
    """Evaluate boolean point-in-box-union occupancy for Nx3 coordinates in millimetres."""
    assert points_mm.ndim == 2 and points_mm.shape[1] == 3, "points must be Nx3"
    assert len(rows) > 0, "empty box list"
    mask = np.zeros(len(points_mm), dtype=bool)
    for _, center, half in rows:
        mask |= np.all(np.abs(points_mm - center) < (half - TOL_MM), axis=1)
    return mask


def d4_asymmetry_count(rows: List[Tuple[str, np.ndarray, np.ndarray]]) -> int:
    """Count cell midpoints whose box occupancy violates any 8-fold D4 XY symmetry."""
    assert len(rows) > 0, "empty box list"
    coords: List[np.ndarray] = []
    for axis in range(3):
        bounds = np.concatenate([[c[axis] - h[axis], c[axis] + h[axis]] for _, c, h in rows])
        uniq = np.unique(np.round(bounds, 4))
        assert len(uniq) >= 2, f"degenerate box bounds along axis {axis}"
        coords.append(0.5 * (uniq[:-1] + uniq[1:]))
    grid = np.stack(np.meshgrid(*coords, indexing="ij"), axis=-1).reshape(-1, 3)
    base = union_mask_at(grid, rows)
    transforms = (
        (-grid[:, 0], grid[:, 1]), (grid[:, 0], -grid[:, 1]),
        (-grid[:, 0], -grid[:, 1]), (grid[:, 1], grid[:, 0]),
        (-grid[:, 1], grid[:, 0]), (grid[:, 1], -grid[:, 0]),
        (-grid[:, 1], -grid[:, 0]),
    )
    mismatches = 0
    for tx, ty in transforms:
        warped = np.column_stack([tx, ty, grid[:, 2]])
        mismatches = max(mismatches, int(np.count_nonzero(base != union_mask_at(warped, rows))))
    return mismatches


def _load_light_cube():
    """Dynamically import experiments/righting/light_cube.py without leaking sys.path."""
    righting_dir = _common.ROOT / "experiments" / "righting"
    module_path = righting_dir / "light_cube.py"
    assert module_path.is_file(), f"missing generator script: {module_path}"
    spec = importlib.util.spec_from_file_location("righting_light_cube", module_path)
    assert spec is not None and spec.loader is not None, "failed to load light_cube spec"
    module = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(righting_dir))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(righting_dir))
    return module


class CubeSurfaceGeometry(unittest.TestCase):
    def test_row_counts_materials_and_envelope(self) -> None:
        rows = load_boxes()
        materials = [m for m, _, _ in rows]
        self.assertEqual(len(rows), 74)
        self.assertEqual(materials.count("rubber"), 8)
        self.assertEqual(materials.count("shell"), 66)
        mins = np.min([c - h for _, c, h in rows], axis=0)
        maxs = np.max([c + h for _, c, h in rows], axis=0)
        np.testing.assert_allclose(mins, [-22.15, -22.15, -22.15], atol=1e-4)
        np.testing.assert_allclose(maxs, [22.15, 22.15, 22.15], atol=1e-4)

    def test_no_pairwise_overlap_and_exact_d4_symmetry(self) -> None:
        rows = load_boxes()
        self.assertLessEqual(max_pairwise_overlap_mm(rows), 1e-6)
        rubber = [r for r in rows if r[0] == "rubber"]
        shell = [r for r in rows if r[0] == "shell"]
        self.assertEqual(d4_asymmetry_count(rubber), 0)
        self.assertEqual(d4_asymmetry_count(shell), 0)
        vols = np.array([8.0 * float(np.prod(h)) for _, _, h in rows])
        centers = np.array([c for _, c, _ in rows])
        centroid_xy = np.sum(centers[:, :2] * vols[:, None], axis=0) / np.sum(vols)
        np.testing.assert_allclose(centroid_xy, [0.0, 0.0], atol=1e-6)

    def test_optional_sdk_cube_voxel_fidelity(self) -> None:
        if not SDK_CUBE_PATH.is_file():
            self.skipTest(f"{_common.OPTIONAL_SKIP_PREFIX}missing web/public/robot/sdk/cube.obj")
        light_cube = _load_light_cube()
        rows = load_boxes()
        _, tris = light_cube.sdk_surface(SDK_CUBE_PATH)
        axis = np.linspace(-0.0215, 0.0215, 22)
        pts = np.stack(np.meshgrid(axis, axis, axis, indexing="ij"), axis=-1).reshape(-1, 3)
        mesh_occ = light_cube.inside(pts, tris)
        box_occ = union_mask_at(pts * 1000.0, rows)
        mismatch = int(np.count_nonzero(mesh_occ != box_occ))
        self.assertGreater(len(pts), 10000)
        self.assertLess(mismatch, 60)

    def test_perturbation_breaks_symmetry_and_overlap(self) -> None:
        rows = load_boxes()
        shifted = [(m, c + np.array([1.0, 0.0, 0.0]), h) if i == 1 else (m, c, h) for i, (m, c, h) in enumerate(rows)]
        shell = [r for r in shifted if r[0] == "shell"]
        self.assertGreater(d4_asymmetry_count(shell), 0)
        self.assertGreater(max_pairwise_overlap_mm(shifted), 0.1)

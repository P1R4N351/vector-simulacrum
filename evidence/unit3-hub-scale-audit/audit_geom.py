"""Photogrammetry and multi-camera calibration primitives for unit 3 hub-scale audit."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

WORLD_DOCK_MM = np.array(
    [
        [-23.0, 0.0, 71.5],
        [23.0, 0.0, 71.5],
        [23.0, 0.0, 25.5],
        [-23.0, 0.0, 25.5],
        [0.0, -94.0, 0.0],
    ],
    dtype=float,
)
SEE_GLYPH_PX = np.array(
    [[437.5, 84.5], [506.5, 84.5], [507.5, 150.5], [438.5, 150.0], [466.0, 256.0]],
    dtype=float,
) + np.array([390.0, 455.0], dtype=float)
SEE_SHELF_PX = np.array(
    [[225.0, 154.0], [729.0, 165.0], [69.0, 263.0], [882.0, 292.0]], dtype=float
) + np.array([390.0, 455.0], dtype=float)
SEE_CROP_OFFSET = np.array([310.0, 395.0], dtype=float)
HUB_Z_NOMINAL = (14.5, 10.75)
PHONE_F_PRIORS = (662.0, 376.0)
PHONE_BASELINE_MM = 14.232


def rectify_uv(uv: np.ndarray, lens: np.ndarray) -> np.ndarray:
    """Undo 1-parameter division radial distortion around principal center."""
    pts = np.asarray(uv, dtype=float)
    assert pts.shape[-1] == 2 and len(lens) == 3, "expected 2D points and 3-elem lens"
    k, center = float(lens[0]), lens[1:]
    delta = pts - center
    r2 = np.sum((delta / 960.0) ** 2, axis=-1, keepdims=True)
    denom = 1.0 + k * r2
    assert np.all(np.abs(denom) > 1e-6), "degenerate division rectification"
    return center + delta / denom


def distort_uv(uv: np.ndarray, lens: np.ndarray) -> np.ndarray:
    """Apply inverse division radial distortion around principal center."""
    pts = np.asarray(uv, dtype=float)
    assert pts.shape[-1] == 2 and len(lens) == 3, "expected 2D points and 3-elem lens"
    k, center = float(lens[0]), lens[1:]
    delta = pts - center
    disc = np.maximum(1e-9, 1.0 - 4.0 * k * np.sum((delta / 960.0) ** 2, axis=-1, keepdims=True))
    assert np.all(np.isfinite(disc)), "non-finite distortion radicand"
    return center + 2.0 * delta / (1.0 + np.sqrt(disc))


def fit_see_lens(edge_csv: Path, ids: Sequence[int] = (0, 1, 2)) -> np.ndarray:
    """Fit 1-parameter division lens distortion from straight shelf edge pixels."""
    assert edge_csv.is_file(), f"missing edge csv: {edge_csv}"
    data = np.loadtxt(edge_csv, delimiter=",", skiprows=1)
    assert data.ndim == 2 and data.shape[1] >= 3, "invalid edge csv columns"
    groups = [data[data[:, 0] == idx, 1:3] for idx in range(7)]

    def _seed(line: np.ndarray) -> List[float]:
        d = (line - np.array([960.0, 540.0])) / 960.0
        n = np.linalg.svd(d - d.mean(axis=0), full_matrices=False)[2][-1]
        return [float(math.atan2(n[1], n[0])), float(d.mean(axis=0) @ n)]

    def _err(q: np.ndarray) -> np.ndarray:
        chunks = []
        for j, idx in enumerate(ids):
            th, b = q[3 + 2 * j], q[4 + 2 * j]
            n = np.array([math.cos(th), math.sin(th)])
            d = (groups[idx] - q[1:3]) / 960.0
            denom = np.linalg.norm(n - 2.0 * b * q[0] * d, axis=1)
            chunks.append(960.0 * (d @ n - b * (1.0 + q[0] * (d * d).sum(axis=1))) / denom)
        return np.r_[np.concatenate(chunks), (q[1:3] - np.array([960.0, 540.0])) / 30.0]

    seed = [-0.75, 960.0, 540.0] + [v for idx in ids for v in _seed(groups[idx])]
    return least_squares(_err, seed, loss="soft_l1").x[:3]


def project_pinhole(xyz: np.ndarray, pose: np.ndarray, pp: Sequence[float]) -> np.ndarray:
    """Project 3D world points into undistorted pinhole pixel coordinates."""
    pts = np.atleast_2d(np.asarray(xyz, dtype=float))
    assert pts.shape[1] == 3 and len(pose) == 7, "expected (N,3) points and 7-param pose"
    cam = Rotation.from_rotvec(pose[:3]).apply(pts) + pose[3:6]
    z_safe = np.where(np.abs(cam[:, 2:3]) > 1e-4, cam[:, 2:3], 1e-4)
    assert np.all(np.isfinite(cam)), "non-finite camera coordinates"
    return cam[:, :2] / z_safe * float(pose[6]) + np.asarray(pp, dtype=float)


def project_see_raw(xyz: np.ndarray, pose: np.ndarray, lens: np.ndarray) -> np.ndarray:
    """Project 3D world points into distorted native See3CAM image coordinates."""
    pts = np.atleast_2d(np.asarray(xyz, dtype=float))
    assert pts.shape[1] == 3 and len(pose) == 7, "expected (N,3) points and 7-param pose"
    return distort_uv(project_pinhole(pts, pose, lens[1:]), lens)


def camera_center(pose: np.ndarray) -> np.ndarray:
    """Compute camera optical center in world coordinates."""
    assert len(pose) == 7, "expected 7-parameter camera pose"
    c = Rotation.from_rotvec(-pose[:3]).apply(-pose[3:6])
    assert np.all(np.isfinite(c)), "non-finite camera center"
    return c


def camera_ray(
    uv: np.ndarray, pose: np.ndarray, pp: Sequence[float]
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute world ray origin and unit direction from undistorted pixel uv."""
    pt = np.asarray(uv, dtype=float)
    assert pt.shape == (2,) and len(pose) == 7, "expected 2D pixel and 7-param pose"
    inv = Rotation.from_rotvec(-pose[:3])
    vec = inv.apply(np.r_[(pt - np.asarray(pp, dtype=float)) / float(pose[6]), 1.0])
    return inv.apply(-pose[3:6]), vec / float(np.linalg.norm(vec))


def intersect_plane_z(origin: np.ndarray, direction: np.ndarray, z_mm: float) -> np.ndarray:
    """Intersect a 3D ray with a horizontal plane at height z_mm."""
    assert abs(float(direction[2])) > 1e-6, "ray nearly parallel to horizontal z plane"
    t = (float(z_mm) - float(origin[2])) / float(direction[2])
    assert math.isfinite(t), "invalid ray-plane parameter"
    return origin + t * direction


def intersect_vertical_plane(
    origin: np.ndarray, direction: np.ndarray, p0: np.ndarray, normal: np.ndarray
) -> np.ndarray:
    """Intersect a 3D ray with an arbitrary plane defined by point p0 and unit normal."""
    denom = float(direction @ normal)
    assert abs(denom) > 1e-6, "ray nearly parallel to vertical wheel plane"
    t = float((p0 - origin) @ normal) / denom
    assert math.isfinite(t), "invalid vertical plane parameter"
    return origin + t * direction


def triangulate_rays(rays: Sequence[Tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
    """Compute least-squares closest point intersection of two or more 3D rays."""
    assert len(rays) >= 2, "at least 2 rays required for triangulation"
    mats = [np.eye(3) - np.outer(d, d) for _, d in rays]
    rhs = sum(m @ o for m, (o, _) in zip(mats, rays))
    pt = np.linalg.solve(sum(mats), rhs)
    assert np.all(np.isfinite(pt)), "triangulation produced non-finite coordinates"
    return pt


def detect_glyph_quad(img_bgr: np.ndarray, is_ultra: bool) -> np.ndarray:
    """Detect four outer rounded-square corners of the charger glyph in 1000x750 image."""
    assert img_bgr is not None and img_bgr.ndim == 3, "expected valid BGR image"
    small = cv2.resize(img_bgr, (1000, 750))
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    cen = np.array([598.0, 387.0]) if is_ultra else np.array([693.0, 378.0])
    ww = 52.0 if is_ultra else 95.0
    found = _collect_glyph_candidates(gray, cen, ww)
    assert len(found) > 0, "dock glyph contour not detected"
    return np.median(found, axis=0)


def _collect_glyph_candidates(gray: np.ndarray, cen: np.ndarray, ww: float) -> List[np.ndarray]:
    assert gray.ndim == 2 and ww > 10.0, "invalid glyph detection inputs"
    out: List[np.ndarray] = []
    for thresh in (125, 135, 145, 155, 165, 175, 185, 195, 205):
        cnts, _ = cv2.findContours((gray > thresh).astype(np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        for co in cnts:
            x, y, w, h = cv2.boundingRect(co)
            mid = np.array([x + w / 2.0, y + h / 2.0])
            ok = np.linalg.norm(mid - cen) <= 30.0 and abs(w - ww) <= 0.12 * ww
            if ok and 0.72 < (w / max(h, 1)) < 1.12 and cv2.contourArea(co) >= 0.82 * w * h:
                out.append(_fit_quad_corners(co[:, 0, :], mid, float(w), float(h)))
    assert len(out) <= 50, "unexpectedly many glyph candidates"
    return out


def _fit_quad_corners(pts: np.ndarray, mid: np.ndarray, w: float, h: float) -> np.ndarray:
    assert len(pts) >= 12, "insufficient contour points for quad fit"
    normed = (pts - mid) / np.array([w, h])
    masks = [
        (np.abs(normed[:, 0]) < 0.30) & (normed[:, 1] < -0.34),
        (np.abs(normed[:, 0]) < 0.30) & (normed[:, 1] > 0.34),
        (np.abs(normed[:, 1]) < 0.30) & (normed[:, 0] < -0.34),
        (np.abs(normed[:, 1]) < 0.30) & (normed[:, 0] > 0.34),
    ]
    sides = []
    for mask in masks:
        v = cv2.fitLine(pts[mask], cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        sides.append(np.cross(np.r_[v[2:], 1.0], np.r_[v[2:] + v[:2], 1.0]))
    xy = [np.cross(sides[i], sides[j]) for i, j in ((0, 2), (0, 3), (1, 3), (1, 2))]
    corners = np.array([p[:2] / p[2] for p in xy], dtype=float)
    assert corners.shape == (4, 2) and np.all(np.isfinite(corners)), "invalid quad corners"
    return corners


def seed_phone_pose(glyph_uv: np.ndarray, is_ultra: bool, lip_uv: np.ndarray) -> np.ndarray:
    """Solve initial 7-DOF phone pose from 46 mm dock corners and optional front lip."""
    assert glyph_uv.shape == (4, 2) and np.asarray(lip_uv).shape == (2,), "invalid seed points"
    f0 = 376.0 if is_ultra else 662.0
    k_mat = np.array([[f0, 0.0, 500.0], [0.0, f0, 375.0], [0.0, 0.0, 1.0]])
    obs = np.vstack((glyph_uv, lip_uv))
    ok, rvec, tvec = cv2.solvePnP(WORLD_DOCK_MM, obs, k_mat, None, flags=cv2.SOLVEPNP_EPNP)
    assert ok, "initial phone PnP failed"

    def _err(p: np.ndarray) -> np.ndarray:
        diff = (project_pinhole(WORLD_DOCK_MM, p, (500.0, 375.0)) - obs) / np.array([[0.65]] * 4 + [[3.0]])
        return np.r_[diff.ravel(), (p[6] - f0) / (0.04 * f0)]

    return least_squares(_err, np.r_[rvec.ravel(), tvec.ravel(), f0]).x


def fit_see_pose_only(
    lens: np.ndarray, glyph_mm: float = 46.0, use_shelf: bool = True, fprior: Optional[float] = 580.0
) -> np.ndarray:
    """Fit See3CAM 7-DOF pose with selectable shelf orthogonality and focal prior."""
    assert len(lens) == 3 and glyph_mm > 20.0, "invalid See3CAM pose fit inputs"
    world = scaled_world(glyph_mm, -94.0)

    def _locate0(uv: np.ndarray, p: np.ndarray) -> np.ndarray:
        rays = (rectify_uv(uv, lens) - lens[1:]) / float(p[6])
        inv = Rotation.from_rotvec(-p[:3])
        ori, dirs = inv.apply(-p[3:6]), inv.apply(np.c_[rays, np.ones(len(rays))])
        return (ori + (-ori[2]) * dirs / dirs[:, 2:3])[:, :2]

    def _err(p: np.ndarray) -> np.ndarray:
        parts = [((project_see_raw(world, p, lens) - SEE_GLYPH_PX) / 1.5).ravel()]
        if fprior is not None:
            parts.append(np.array([(p[6] - fprior) / 150.0]))
        if use_shelf:
            a, b, c_pt, d_pt = _locate0(SEE_SHELF_PX, p)
            across = (b - a) / np.linalg.norm(b - a)
            parts.append(np.array([((c_pt - a) @ across) / 5.0, ((d_pt - b) @ across) / 5.0]))
        return np.concatenate(parts)

    lb = np.r_[[-7.0] * 3, [-2000.0] * 2, 50.0, 250.0]
    ub = np.r_[[7.0] * 3, [2000.0] * 3, 1400.0]
    return least_squares(_err, [1.8, -0.08, 0.12, -35.0, 71.0, 389.0, 595.0], bounds=(lb, ub)).x


def scaled_world(glyph_mm: float, lip_y_mm: float) -> np.ndarray:
    """Return 5-point dock world coordinates scaled around glyph center (0, 0, 48.5)."""
    assert 30.0 <= glyph_mm <= 65.0 and -120.0 <= lip_y_mm <= -70.0, "implausible dock bounds"
    sc = float(glyph_mm) / 46.0
    world = WORLD_DOCK_MM.copy()
    world[:4, 0] *= sc
    world[:4, 2] = 48.5 + (world[:4, 2] - 48.5) * sc
    world[4, 1] = float(lip_y_mm)
    return world


def extract_fcc_jpegs(pdf_path: Path) -> List[np.ndarray]:
    """Decode the 4 1200x900 embedded JPEGs from DDL Vector FCC external photos PDF."""
    assert pdf_path.is_file(), f"missing FCC PDF: {pdf_path}"
    blob = pdf_path.read_bytes()
    out: List[np.ndarray] = []
    pos = 0
    for _ in range(32):
        start = blob.find(b"\xff\xd8\xff", pos)
        if start < 0:
            break
        end = blob.find(b"\xff\xd9", start)
        if end < 0:
            break
        arr = np.frombuffer(blob[start : end + 2], dtype=np.uint8)
        im = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if im is not None and im.shape[:2] == (900, 1200):
            out.append(im)
        pos = end + 2
    assert len(out) == 4, f"expected 4 1200x900 JPEGs in {pdf_path}, got {len(out)}"
    return out

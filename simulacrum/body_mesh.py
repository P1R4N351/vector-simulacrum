"""Vector's real body shape, cut from Anki's animation rig into the simulacrum's moving parts.

The parametric model (mjcf.py) draws the robot as boxes and cylinders sized from the vendor's
Webots model. This module builds the robot's actual outer shape instead, from the rig Anki's
animators used (Victor_rig_01.obj in kercre123/vector-animations-raw), whose joints land on
the firmware's own constants: front and rear axle heights 14.51 and 10.77 mm (firmware 14.5
and 10.75), wheel base 47.99 mm (48), upper lift pivot (-41.11, 45.03) mm (-41, 45), neck
pivot (-12.92, 49.71) mm (-13, 49). `build` re-measures every one of these and refuses a rig
that has drifted from the table.

The rig is NOT part of this repository: its repository carries no licence (NOASSERTION), so
`fetch` downloads it at a pinned commit and hash and `build` writes the derived meshes to a
cache outside the tree (default ~/.cache/vector-simulacrum/body/). mjcf.build_mjcf(v, body=...)
uses them when asked; without them the parametric model is unchanged.

What `build` writes, per moving part (chassis, head, upper lift links, lower lift links, fork):
  * collision hulls: convex pieces in the part's body frame (CoACD when importable, else one
    hull per rig group), each scored against the rig surface by `fidelity`;
  * one visual mesh: the rig's own triangles in the same frame.
The lower lift links become their own body in the model (the real lift is a parallelogram; the
parametric model has only the upper link). Masses stay where the parameter table puts them:
the hulls carry contact, not mass.

  python3 -m simulacrum.body_mesh fetch [--out DIR]
  python3 -m simulacrum.body_mesh build [--rig FILE] [--out DIR] [--method coacd|groups]
  python3 -m simulacrum.body_mesh check [--out DIR]       fidelity report of a built body
  python3 -m simulacrum.body_mesh rescore [--out DIR]     score an existing build again, no decomposition
Exit: 0 done, 1 check failed, 2 bad arguments, 3 rig missing, 4 hash mismatch, 5 rig drifted.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

RIG = {
    "repo": "kercre123/vector-animations-raw",
    "commit": "59b198d4e6fb7394ae4c3d832dea2c4d28866ab0",
    "path": "assets/rigs/Base/obj/Victor_rig_01.obj",
    "sha256": "59890fafc1c122726d73f49fab66913d45d87426143107d9359e6ca81e7fad60",
    "bytes": 13309832,
}
RIG_URL = f"https://raw.githubusercontent.com/{RIG['repo']}/{RIG['commit']}/{RIG['path']}"
RIG_FILE = "Victor_rig_01.obj"
DEFAULT_OUT = Path.home() / ".cache" / "vector-simulacrum" / "body"
MANIFEST = "manifest.json"
MAX_OBJ_LINES = 2_000_000
DOWNLOAD_TIMEOUT_S = 300
CM_TO_MM = 10.0
# How far a rig feature may sit from the table's value before build refuses (mm).
FEATURE_TOLERANCE_MM = 1.0
# Hull pieces that rest on the table plane are lifted to this height so the wheels keep
# carrying the robot on a flat table, as mjcf._tip_supports_xml does for its cylinders.
TREAD_HULL_CLEARANCE_MM = 0.3
COACD_THRESHOLD = 0.02
COACD_SEED = 20261122
PARTS = ("chassis", "head", "lift_upper", "lift_lower", "fork")
EXIT_FAILED, EXIT_USAGE, EXIT_MISSING, EXIT_HASH, EXIT_DRIFT = 1, 2, 3, 4, 5

# Rig group name -> rig piece. First match wins; None drops the group (the rig carries a
# second, low-resolution copy of every part under LP_ / low_res_grp).
PIECE_RULES = (
    ("LP_", None), ("low_res_grp", None),
    ("Eye_rig", "face"), ("overscan", "face"), ("head_grp", "head"),
    ("Upper_Hand", "arm_upper"), ("Lower_Hand", "arm_lower"),
    ("Front_Mid_Part", "fork"), ("Back_Mid_Part", "fork"), ("Bottom_Mid_Part", "fork"),
    ("Rubber_Chain", "tread"), ("Wheels", "wheel"), ("Tooth", "wheel"),
    ("body_grp", "body"), ("backpack", "body"),
)
# Rig piece -> model part, and whether the piece's own surface is a collision source.
PIECE_PART = {"body": ("chassis", True), "tread": ("chassis", False), "wheel": ("chassis", False),
              "head": ("head", True), "face": ("head", False), "arm_upper": ("lift_upper", True),
              "arm_lower": ("lift_lower", True), "fork": ("fork", True)}


def piece_of(group: str) -> Optional[str]:
    """Which rig piece a group belongs to."""
    if group.endswith("L_Wheels L_Tooth"):  # the left rubber chain carries this name in the rig
        return "tread"
    for key, piece in PIECE_RULES:
        if key in group:
            return piece
    raise ValueError(f"rig group not classified: {group!r}")


# ---- reading -----------------------------------------------------------------------------

def read_obj(path: Path) -> Tuple[np.ndarray, List[Tuple[str, np.ndarray]]]:
    """Vertices (n, 3) in file units and, per face group, its triangles (fan-split polygons)."""
    verts: List[List[float]] = []
    groups: Dict[str, List[List[int]]] = {}
    current = "default"
    with path.open(errors="replace") as source:
        for count, line in enumerate(source):
            assert count < MAX_OBJ_LINES, "OBJ longer than the rig"
            if line.startswith("v "):
                verts.append([float(t) for t in line.split()[1:4]])
            elif line.startswith(("g ", "o ")):
                current = line[2:].strip() or "default"
            elif line.startswith("f "):
                idx = [int(t.split("/")[0]) - 1 for t in line.split()[1:]]
                assert len(idx) >= 3 and min(idx) >= 0, "face needs three positive indices"
                tris = groups.setdefault(current, [])
                tris.extend([idx[0], idx[k], idx[k + 1]] for k in range(1, len(idx) - 1))
    v = np.asarray(verts, dtype=float)
    assert v.ndim == 2 and v.shape[1] == 3 and len(v) > 0, "no vertices"
    out = [(name, np.asarray(t, dtype=np.int64)) for name, t in groups.items()]
    assert all(t.max() < len(v) for _, t in out), "face index past the vertex list"
    return v, out


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for _ in range(1 << 20):
            block = source.read(1 << 20)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def fetch(out: Path) -> Path:
    """Download the rig at the pinned commit into out/, checked against the pinned hash."""
    out.mkdir(parents=True, exist_ok=True)
    dest = out / RIG_FILE
    if dest.is_file() and sha256_file(dest) == RIG["sha256"]:
        return dest
    part = dest.with_suffix(".part")
    with urllib.request.urlopen(RIG_URL, timeout=DOWNLOAD_TIMEOUT_S) as response, part.open("wb") as sink:
        for _ in range(1 << 16):
            block = response.read(1 << 20)
            if not block:
                break
            sink.write(block)
    if sha256_file(part) != RIG["sha256"]:
        part.unlink()
        raise RuntimeError("downloaded rig does not match the pinned sha256")
    part.replace(dest)
    return dest


# ---- registration: rig centimetres to robot millimetres --------------------------------------

def circle_fit(xy: np.ndarray) -> Tuple[np.ndarray, float, float]:
    """Least-squares circle (Kasa): centre, radius, rms radial residual."""
    assert xy.ndim == 2 and xy.shape[1] == 2 and len(xy) >= 3, "circle fit needs three points"
    a = np.column_stack((2.0 * xy[:, 0], 2.0 * xy[:, 1], np.ones(len(xy))))
    sol, *_ = np.linalg.lstsq(a, (xy ** 2).sum(1), rcond=None)
    centre = sol[:2]
    radius = float(math.sqrt(max(sol[2] + centre @ centre, 0.0)))
    resid = np.hypot(*(xy - centre).T) - radius
    return centre, radius, float(np.sqrt((resid ** 2).mean()))


def end_cap(xz: np.ndarray, front: bool, rounds: int = 8) -> Dict[str, Any]:
    """Centre of the rounded end of a lift link: fit the half-ring beyond the centre, iterated."""
    edge = xz[:, 0].max() if front else xz[:, 0].min()
    sel = (xz[:, 0] > edge - 5.0) if front else (xz[:, 0] < edge + 5.0)
    centre, radius, rms = circle_fit(xz[sel])
    for _ in range(rounds):
        beyond = (xz[:, 0] >= centre[0]) if front else (xz[:, 0] <= centre[0])
        sel = beyond & (np.abs(np.hypot(*(xz - centre).T) - radius) < 0.6)
        assert sel.sum() >= 12, "end cap lost its points"
        centre, radius, rms = circle_fit(xz[sel])
    return {"centre_xz": centre.tolist(), "radius": radius, "rms": rms, "n": int(sel.sum())}


def _piece_points(v: np.ndarray, groups: Sequence[Tuple[str, np.ndarray]], keys: Sequence[str]) -> np.ndarray:
    ids = np.unique(np.concatenate([t.ravel() for name, t in groups if any(k in name for k in keys)]))
    assert len(ids) > 0, f"no rig group matches {keys}"
    return v[ids]


def register(v: np.ndarray, groups: Sequence[Tuple[str, np.ndarray]]) -> Dict[str, float]:
    """Rig frame -> robot frame. Rig: Maya cm, +Z forward, +X robot left, +Y up.
    Robot: mm, origin on the table under the front axle, x forward, y left, z up."""
    tread = np.concatenate([v[np.unique(t)] for name, t in groups if piece_of(name) == "tread"])
    ground_y = float(tread[:, 1].min())
    front = _piece_points(v, groups, ("R_Large_Ri", "L_Tooth21"))
    centre, _, _ = circle_fit(front[:, [2, 1]])
    rims_r = _piece_points(v, groups, ("R_Large_Ri", "R_Small_Ri"))
    rims_l = _piece_points(v, groups, ("L_Tooth21", "L_Tooth20"))
    # The rig sits 0.36 mm off the robot's centre line; centre it between the two outer rims.
    mid = 0.25 * (rims_r[:, 0].min() + rims_r[:, 0].max() + rims_l[:, 0].min() + rims_l[:, 0].max())
    return {"ground_y_cm": ground_y, "front_axle_z_cm": float(centre[0]), "centre_x_cm": float(mid)}


def to_robot(p_cm: np.ndarray, reg: Dict[str, float]) -> np.ndarray:
    assert p_cm.ndim == 2 and p_cm.shape[1] == 3, "points are (n, 3)"
    return np.column_stack(((p_cm[:, 2] - reg["front_axle_z_cm"]) * CM_TO_MM,
                            (p_cm[:, 0] - reg["centre_x_cm"]) * CM_TO_MM,
                            (p_cm[:, 1] - reg["ground_y_cm"]) * CM_TO_MM))


def measure(v_mm: np.ndarray, groups: Sequence[Tuple[str, np.ndarray]]) -> Dict[str, Any]:
    """The rig's joints and wheels, in robot mm."""
    pts = lambda *keys: _piece_points(v_mm, groups, keys)
    out: Dict[str, Any] = {}
    for label, keys in (("front_rims", ("R_Large_Ri", "L_Tooth21")), ("rear_rims", ("R_Small_Ri", "L_Tooth20")),
                        ("head_side_frames", ("R_Large_Frame", "L_Large_Frame"))):
        c, r, e = circle_fit(pts(*keys)[:, [0, 2]])
        out[label] = {"centre_xz": c.tolist(), "radius": r, "rms": e}
    for label, key in (("upper", "_Upper_Hand"), ("lower", "_Lower_Hand")):
        xz = pts(key)[:, [0, 2]]
        out[f"{label}_link_rear"] = end_cap(xz, front=False)
        out[f"{label}_link_front"] = end_cap(xz, front=True)
    tread = np.concatenate([v_mm[np.unique(t)] for n, t in groups if piece_of(n) == "tread"])
    for side, sign in (("left", 1.0), ("right", -1.0)):
        band = tread[(np.sign(tread[:, 1]) == sign) & (tread[:, 2] < 1.0)]
        out[f"tread_contact_{side}_y"] = [float(band[:, 1].min()), float(band[:, 1].max())]
    cam = pts("Mouth")
    lens = cam[cam[:, 0] > cam[:, 0].max() - 1.0]
    out["camera_front_centre_xz"] = [float(lens[:, 0].mean()), float(0.5 * (lens[:, 2].min() + lens[:, 2].max()))]
    return out


# ---- cutting the rig into the model's moving parts ------------------------------------------

def rot_y_up(points: np.ndarray, angle: float) -> np.ndarray:
    """Turn points about the model's hinge axis (0, -1, 0) by angle: positive raises +x towards +z."""
    c, s = math.cos(angle), math.sin(angle)
    x, z = points[:, 0], points[:, 2]
    return np.column_stack((c * x - s * z, points[:, 1], s * x + c * z))


def check_against_table(v: Dict[str, Any], feats: Dict[str, Any]) -> List[str]:
    """Where the rig's joints and wheels disagree with the parameter table by more than the tolerance."""
    neck, base = v["neck_joint_position_mm"], v["lift_base_position_mm"]
    pairs = [
        ("front axle height", feats["front_rims"]["centre_xz"][1], v["wheel_front_radius_mm"]),
        ("rear axle x", feats["rear_rims"]["centre_xz"][0], -v["wheel_base_mm"]),
        ("rear axle height", feats["rear_rims"]["centre_xz"][1], v["wheel_rear_radius_mm"]),
        ("neck x", feats["head_side_frames"]["centre_xz"][0], neck[0]),
        ("neck z", feats["head_side_frames"]["centre_xz"][1], neck[2]),
        ("lift pivot x", feats["upper_link_rear"]["centre_xz"][0], base[0]),
        ("lift pivot z", feats["upper_link_rear"]["centre_xz"][1], base[2]),
        ("lift link length", link_length(feats, "lower"), v["lift_arm_length_mm"]),
    ]
    return [f"{name}: rig {got:.2f} mm, table {want:.2f} mm" for name, got, want in pairs
            if abs(got - want) > FEATURE_TOLERANCE_MM]


def link_length(feats: Dict[str, Any], which: str) -> float:
    rear, front = feats[f"{which}_link_rear"]["centre_xz"], feats[f"{which}_link_front"]["centre_xz"]
    return float(math.hypot(front[0] - rear[0], front[1] - rear[1]))


def rig_lift_angle(feats: Dict[str, Any]) -> float:
    """The lift angle the rig was saved at, from the two links' directions (they are parallel)."""
    angles = []
    for which in ("upper", "lower"):
        rear, front = feats[f"{which}_link_rear"]["centre_xz"], feats[f"{which}_link_front"]["centre_xz"]
        angles.append(math.atan2(front[1] - rear[1], front[0] - rear[0]))
    assert abs(angles[0] - angles[1]) < math.radians(2.0), "lift links are not parallel in the rig"
    return 0.5 * (angles[0] + angles[1])


def part_frames(v: Dict[str, Any], feats: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    """Each part's body origin (robot mm, the table's joint positions) and the joint angle the rig
    holds it at. The lower links pivot below the upper ones by the rig's own offset."""
    base = np.asarray(v["lift_base_position_mm"], dtype=float)
    a0 = rig_lift_angle(feats)
    # The lower links turn about their own rear cap: offset it from the upper rear cap by what the rig
    # shows (fitted half-rings, rms about 0.2 mm). The rig is not an exact parallelogram: its front caps
    # sit 0.3 mm further apart, and the two links are 65.05 and 65.39 mm long.
    ru, rl = feats["upper_link_rear"]["centre_xz"], feats["lower_link_rear"]["centre_xz"]
    lower = base + np.array([rl[0] - ru[0], 0.0, rl[1] - ru[1]])
    wrist = base + v["lift_arm_length_mm"] * np.array([math.cos(a0), 0.0, math.sin(a0)])
    return {
        "chassis": {"origin_mm": [0.0, 0.0, 0.0], "rig_angle_rad": 0.0},
        "head": {"origin_mm": list(map(float, v["neck_joint_position_mm"])), "rig_angle_rad": 0.0},
        "lift_upper": {"origin_mm": base.tolist(), "rig_angle_rad": a0},
        "lift_lower": {"origin_mm": lower.tolist(), "rig_angle_rad": a0},
        "fork": {"origin_mm": wrist.tolist(), "rig_angle_rad": 0.0},
    }


def to_part(points_mm: np.ndarray, frame: Dict[str, Any]) -> np.ndarray:
    """Robot-frame points at the rig's pose -> the part's body frame at joint angle zero."""
    local = points_mm - np.asarray(frame["origin_mm"])
    return rot_y_up(local, -frame["rig_angle_rad"])


def split_parts(v_mm: np.ndarray, groups: Sequence[Tuple[str, np.ndarray]],
                frames: Dict[str, Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    """Per part: (vertices, triangles) of its visual surface, and of its collision source surface,
    plus each tread side's points for the chassis's tread hulls. All in body frames."""
    out = {p: {"visual": [], "collide": [], "plug": [], "tread": {"left": [], "right": []}} for p in PARTS}
    for name, tris in groups:
        piece = piece_of(name)
        if piece is None:
            continue
        part, collides = PIECE_PART[piece]
        ids, local_tris = np.unique(tris, return_inverse=True)
        local = to_part(v_mm[ids], frames[part])
        entry = (name, local, local_tris.reshape(-1, 3))
        out[part]["visual"].append(entry)
        if collides:
            out[part]["collide"].append(entry)
        if piece in ("tread", "wheel"):
            side = "left" if local[:, 1].mean() > 0.0 else "right"
            out[part]["tread"][side].append(local)
            out["chassis"]["plug"].append(entry)
        elif piece == "head":  # the head closes the chassis's open front while the chassis is filled
            out["chassis"]["plug"].append((name, v_mm[ids], local_tris.reshape(-1, 3)))
    return out


def merge(entries: Sequence[Tuple[str, np.ndarray, np.ndarray]]) -> Tuple[np.ndarray, np.ndarray]:
    """Concatenate (name, verts, tris) entries into one mesh."""
    verts, tris, offset = [], [], 0
    for _, p, t in entries:
        verts.append(p)
        tris.append(t + offset)
        offset += len(p)
    assert offset > 0, "nothing to merge"
    return np.concatenate(verts), np.concatenate(tris)


# ---- convex pieces ----------------------------------------------------------------------------

def hull(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Convex hull as (vertices, outward triangles)."""
    from scipy.spatial import ConvexHull  # build-time dependency only

    h = ConvexHull(points)
    v = points[h.vertices]
    remap = {int(old): new for new, old in enumerate(h.vertices)}
    tris = np.array([[remap[int(i)] for i in s] for s in h.simplices], dtype=np.int64)
    centre = v.mean(0)
    for k, (a, b, c) in enumerate(tris):  # orient every face outward
        n = np.cross(v[b] - v[a], v[c] - v[a])
        if n @ (v[a] - centre) < 0.0:
            tris[k] = (a, c, b)
    return v, tris


def mesh_volume(verts: np.ndarray, tris: np.ndarray) -> float:
    """Volume enclosed by a closed, outward triangle mesh (divergence theorem)."""
    a, b, c = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


def clipped_hull(points: np.ndarray, floor_mm: float) -> Tuple[np.ndarray, np.ndarray]:
    """Convex hull of points intersected with z >= floor_mm (exactly, by half-space intersection)."""
    from scipy.spatial import ConvexHull, HalfspaceIntersection

    h = ConvexHull(points)
    spaces = np.vstack((h.equations, [[0.0, 0.0, -1.0, floor_mm]]))
    inside = points.mean(0)
    inside[2] = max(inside[2], floor_mm + 1.0)
    assert np.all(spaces[:, :3] @ inside + spaces[:, 3] < 0.0), "interior point is not inside"
    return hull(HalfspaceIntersection(spaces, inside).intersections)


VOXEL_MM = 0.5
CLOSE_MM = 1.5
MAX_VOXELS = 8_000_000
# Most convex pieces CoACD may return per part: enough for the shape, few enough for a fast step.
PIECE_BUDGET = {"chassis": 24, "head": 10, "lift_upper": 8, "lift_lower": 12, "fork": 8}


def grid_for(entries: Sequence[Tuple[str, np.ndarray, np.ndarray]], margin_mm: float) -> Tuple[np.ndarray, Tuple[int, int, int]]:
    """Origin (mm) and shape of a VOXEL_MM grid round some surfaces."""
    pts = np.concatenate([p for _, p, _ in entries])
    origin = pts.min(0) - margin_mm
    shape = tuple(int(n) for n in np.ceil((pts.max(0) + margin_mm - origin) / VOXEL_MM) + 1)
    assert int(np.prod(shape)) <= MAX_VOXELS, "voxel grid too large"
    return origin, shape


def rasterize(entries: Sequence[Tuple[str, np.ndarray, np.ndarray]], origin: np.ndarray,
              shape: Tuple[int, int, int]) -> np.ndarray:
    """Voxels the surfaces pass through."""
    mark = np.zeros(shape, dtype=bool)
    rng = np.random.default_rng(COACD_SEED)
    for _, p, t in entries:
        pts = surface_samples(p, t, 0.4 * VOXEL_MM, rng)
        idx = np.floor((pts - origin) / VOXEL_MM).astype(np.int64)
        ok = np.all((idx >= 0) & (idx < np.asarray(shape)), axis=1)
        mark[tuple(idx[ok].T)] = True
    return mark


def _shift(a: np.ndarray, axis: int, step: int) -> np.ndarray:
    """a moved one cell along axis (step +1 or -1), zero where nothing moved in."""
    out = np.zeros_like(a)
    src = [slice(None)] * a.ndim
    dst = [slice(None)] * a.ndim
    src[axis], dst[axis] = (slice(0, -1), slice(1, None)) if step > 0 else (slice(1, None), slice(0, -1))
    out[tuple(dst)] = a[tuple(src)]
    return out


def ray_votes(mark: np.ndarray) -> np.ndarray:
    """For each voxel, in how many of the six axis directions the surface lies beyond it (0..6)."""
    m = mark.astype(np.uint8)
    votes = np.zeros(mark.shape, dtype=np.uint8)
    for axis in range(3):
        forward = np.maximum.accumulate(m, axis=axis)
        backward = np.flip(np.maximum.accumulate(np.flip(m, axis=axis), axis=axis), axis=axis)
        votes += _shift(forward, axis, +1) + _shift(backward, axis, -1)
    return votes


def fill(mark: np.ndarray, close_mm: float = CLOSE_MM, min_votes: int = 5) -> np.ndarray:
    """Solid enclosed by a surface that may have holes. A voxel is inside when the surface lies beyond
    it in at least min_votes of the six axis directions (a hole lets one direction out, not five; a
    pocket open on two sides stays outside); gaps up to 2*close_mm are then closed and cavities filled."""
    from scipy import ndimage

    solid = mark | (ray_votes(mark) >= min_votes)
    r = max(1, int(round(close_mm / VOXEL_MM)))
    ball = np.linalg.norm(np.indices((2 * r + 1,) * 3) - r, axis=0) <= r + 1e-9
    closed = ndimage.binary_closing(np.pad(solid, r + 1), structure=ball)
    return ndimage.binary_fill_holes(closed)[r + 1:-(r + 1), r + 1:-(r + 1), r + 1:-(r + 1)]


def voxel_surface(solid: np.ndarray, origin: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Closed triangle mesh of a voxel solid's boundary faces, outward, in mm."""
    pad = np.pad(solid, 1).astype(np.int8)
    corners, quads = [], []
    for axis in range(3):
        u, w = (axis + 1) % 3, (axis + 2) % 3
        step = np.diff(pad, axis=axis)
        for sign, outward in ((-1, True), (1, False)):
            hit = np.argwhere(step == sign)
            j, p, q = hit[:, axis], hit[:, u] - 1, hit[:, w] - 1
            quad = []
            for du, dw in ((0, 0), (1, 0), (1, 1), (0, 1)):
                c = np.zeros((len(hit), 3), dtype=np.int64)
                c[:, axis], c[:, u], c[:, w] = j, p + du, q + dw
                quad.append(c)
            quads.append(quad if outward else quad[::-1])
    allc = np.concatenate([np.concatenate(q) for q in quads])
    keys, inverse = np.unique(allc, axis=0, return_inverse=True)
    inverse = inverse.ravel()
    tris, at = [], 0
    for quad in quads:
        n = len(quad[0])
        ids = [inverse[at + k * n: at + (k + 1) * n] for k in range(4)]
        at += 4 * n
        tris.append(np.column_stack((ids[0], ids[1], ids[2])))
        tris.append(np.column_stack((ids[0], ids[2], ids[3])))
    return origin + keys * VOXEL_MM, np.concatenate(tris)


def coacd_pieces(verts: np.ndarray, tris: np.ndarray, budget: int) -> List[np.ndarray]:
    import coacd  # optional build-time dependency

    coacd.set_log_level("error")
    mesh = coacd.Mesh(verts.astype(np.float64), tris.astype(np.int64))
    parts = coacd.run_coacd(mesh, threshold=COACD_THRESHOLD, max_convex_hull=budget, seed=COACD_SEED,
                            preprocess_mode="auto", preprocess_resolution=100)
    assert 0 < len(parts) <= budget, "CoACD returned no pieces or more than its budget"
    return [np.asarray(q[0], dtype=float) for q in parts]


MIN_COMPONENT_FRACTION = 0.02


def clean(solid: np.ndarray) -> np.ndarray:
    """Drop slivers one voxel thin and components under MIN_COMPONENT_FRACTION of the largest: loose
    scraps would otherwise use up CoACD's piece budget and force the real shape into one hull."""
    from scipy import ndimage

    opened = ndimage.binary_opening(solid, iterations=1)
    labels, count = ndimage.label(opened)
    assert count > 0, "the solid vanished when cleaned"
    sizes = ndimage.sum(opened, labels, range(1, count + 1))
    return np.isin(labels, 1 + np.flatnonzero(sizes >= MIN_COMPONENT_FRACTION * sizes.max()))


def part_solid(name: str, part: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    """The part's filled, cleaned voxel solid and the grid origin. The chassis is filled with the head and
    the treads closing its open front and sides, then the head and the treads are carved back out (the
    treads by their whole hulls, one voxel wider, down to the table)."""
    if name != "chassis":
        origin, shape = grid_for(part["collide"], 2.0 * CLOSE_MM + VOXEL_MM)
        return clean(fill(rasterize(part["collide"], origin, shape))), origin
    from scipy import ndimage

    everything = list(part["collide"]) + list(part["plug"])
    origin, shape = grid_for(everything, 2.0 * CLOSE_MM + VOXEL_MM)
    solid = fill(rasterize(everything, origin, shape))
    head = [e for e in part["plug"] if piece_of(e[0]) == "head"]
    solid &= ~ndimage.binary_dilation(fill(rasterize(head, origin, shape)), iterations=1)
    treads = [(*hull(np.concatenate(part["tread"][side])), "tread") for side in ("left", "right")]
    cells = np.argwhere(solid)
    in_tread = inside_any(origin + (cells + 0.5) * VOXEL_MM, plane_sets(treads), tol=VOXEL_MM)
    solid[tuple(cells[in_tread].T)] = False
    return clean(solid), origin


def collision_pieces(name: str, part: Dict[str, Any], method: str) -> List[Tuple[np.ndarray, np.ndarray, str]]:
    """(vertices, triangles, friction class) of every convex collision piece of one part."""
    treads = [(*clipped_hull(np.concatenate(part["tread"][side]), TREAD_HULL_CLEARANCE_MM), "tread")
              for side in ("left", "right") if part["tread"][side]]
    if method == "groups":
        sources = [p for _, p, _ in part["collide"] if len(p) >= 4]
    else:
        solid, origin = part_solid(name, part)
        hull_v = hull(np.concatenate([p for _, p, _ in part["collide"]]))
        # A shell that failed to fill leaves only its walls: a solid far below its hull's volume.
        print(f"{name}: solid {solid.sum() * VOXEL_MM ** 3 / 1000.0:.1f} cm3, convex hull "
              f"{mesh_volume(*hull_v) / 1000.0:.1f} cm3", file=sys.stderr)
        sources = coacd_pieces(*voxel_surface(solid, origin), PIECE_BUDGET[name])
    return [(*hull(p), "chassis") for p in sources] + treads

# ---- fidelity: how far the collision shape and the rig's surface stand apart ------------------

SAMPLE_SPACING_MM = 0.35
MAX_SAMPLES = 1_500_000
INSIDE_TOL_MM = 1e-6


def surface_samples(verts: np.ndarray, tris: np.ndarray, spacing: float, rng: np.random.Generator) -> np.ndarray:
    """Points spread over a triangle mesh by area, about one per spacing^2 (at least one per triangle)."""
    a, b, c = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    counts = np.maximum(1, np.ceil(area / spacing ** 2)).astype(np.int64)
    total = int(counts.sum())
    assert total <= MAX_SAMPLES, "too many surface samples; raise the spacing"
    owner = np.repeat(np.arange(len(tris)), counts)
    u, w = rng.random(total), rng.random(total)
    flip = u + w > 1.0
    u[flip], w[flip] = 1.0 - u[flip], 1.0 - w[flip]
    return a[owner] + u[:, None] * (b - a)[owner] + w[:, None] * (c - a)[owner]


def plane_sets(pieces: Sequence[Tuple[np.ndarray, np.ndarray, str]]) -> List[np.ndarray]:
    """Outward half-space equations (n, d) of each convex piece, n unit, inside where n.x + d <= 0."""
    out = []
    for v, t, _ in pieces:
        n = np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]])
        norm = np.linalg.norm(n, axis=1)
        keep = norm > 1e-12
        n = n[keep] / norm[keep, None]
        out.append(np.column_stack((n, -(n * v[t[keep, 0]]).sum(1))))
    return out


def inside_any(points: np.ndarray, planes: Sequence[np.ndarray], skip: int = -1, chunk: int = 20000,
               tol: float = INSIDE_TOL_MM) -> np.ndarray:
    """Whether each point lies inside any piece other than piece `skip`, or within tol (mm) outside it.
    Chunked to bound memory."""
    hit = np.zeros(len(points), dtype=bool)
    for start in range(0, len(points), chunk):
        block = points[start:start + chunk]
        for k, eq in enumerate(planes):
            if k == skip:
                continue
            hit[start:start + chunk] |= np.all(block @ eq[:, :3].T + eq[:, 3] <= tol, axis=1)
    return hit


def fidelity(ref_verts: np.ndarray, ref_tris: np.ndarray, pieces: Sequence[Tuple[np.ndarray, np.ndarray, str]],
             seed: int = 0) -> Dict[str, Any]:
    """Two one-sided distances between the rig surface and the boundary of the union of the pieces:
    protrusion: rig surface outside every piece, distance to the nearest piece surface (shape the
    collision model is missing); excess: piece-union boundary, distance to the nearest rig surface
    (collision where the real robot has none, e.g. a filled concavity). mm."""
    from scipy.spatial import cKDTree

    rng = np.random.default_rng(seed)
    ref = surface_samples(ref_verts, ref_tris, SAMPLE_SPACING_MM, rng)
    planes = plane_sets(pieces)
    boundary = []
    for k, (v, t, _) in enumerate(pieces):
        s = surface_samples(v, t, SAMPLE_SPACING_MM, rng)
        boundary.append(s[~inside_any(s, planes, skip=k)])
    bnd = np.concatenate(boundary)
    outside = ~inside_any(ref, planes)
    protrusion = np.zeros(len(ref))
    if outside.any():
        protrusion[outside] = cKDTree(bnd).query(ref[outside])[0]
    excess = cKDTree(ref).query(bnd)[0]
    stats = lambda d: {q: round(float(np.percentile(d, p)), 3) for q, p in (("p50", 50), ("p95", 95), ("p99", 99), ("max", 100))}
    return {"pieces": len(pieces), "ref_samples": int(len(ref)), "boundary_samples": int(len(bnd)),
            "outside_fraction": round(float(outside.mean()), 4), "protrusion_mm": stats(protrusion),
            "excess_mm": stats(excess), "spacing_mm": SAMPLE_SPACING_MM}


# Where two parts' pieces meet, the rig (a visual model) has no surface: the head's underside inside
# the body, the recess the head sits in, the body's faces against the treads. A piece's surface lying
# within this distance of another piece is such an interface and is not scored as excess.
INTERFACE_MM = 1.5


def to_robot_frame(points: np.ndarray, frame: Dict[str, Any]) -> np.ndarray:
    """A part's body-frame points back to the robot frame at the rig's pose (inverse of to_part)."""
    return rot_y_up(points, frame["rig_angle_rad"]) + np.asarray(frame["origin_mm"])


def reference_surfaces(v_mm: np.ndarray, groups: Sequence[Tuple[str, np.ndarray]]) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    """Per part, the rig surface its collision pieces stand for, in the robot frame at the rig's pose."""
    entries: Dict[str, list] = {p: [] for p in PARTS}
    for name, tris in groups:
        piece = piece_of(name)
        if piece is None:
            continue
        part, collides = PIECE_PART[piece]
        if collides or piece in ("tread", "wheel"):
            ids, local = np.unique(tris, return_inverse=True)
            entries[part].append((name, v_mm[ids], local.reshape(-1, 3)))
    return {p: merge(e) for p, e in entries.items()}


def robot_fidelity(refs: Dict[str, Tuple[np.ndarray, np.ndarray]],
                   pieces: Dict[str, List[Tuple[np.ndarray, np.ndarray, str]]], seed: int = 0) -> Dict[str, Any]:
    """fidelity() over the whole robot at the rig's pose, reported per part: protrusion of each part's rig
    surface outside every piece of every part; excess of each part's exposed piece surface (outside all
    other pieces by more than INTERFACE_MM) from the nearest rig surface of any part. mm."""
    from scipy.spatial import cKDTree

    rng = np.random.default_rng(seed)
    flat = [(part, v, t, f) for part, ps in pieces.items() for v, t, f in ps]
    planes = plane_sets([(v, t, f) for _, v, t, f in flat])
    ref = {p: surface_samples(*refs[p], SAMPLE_SPACING_MM, rng) for p in PARTS}
    surface, exposed, owner = [], [], []
    for k, (part, v, t, _) in enumerate(flat):
        s = surface_samples(v, t, SAMPLE_SPACING_MM, rng)
        s = s[~inside_any(s, planes, skip=k)]
        surface.append(s)
        keep = ~inside_any(s, planes, skip=k, tol=INTERFACE_MM)
        exposed.append(s[keep])
        owner += [part] * int(keep.sum())
    bnd_tree = cKDTree(np.concatenate(surface))
    ref_tree = cKDTree(np.concatenate(list(ref.values())))
    exp = np.concatenate(exposed)
    owner = np.asarray(owner)
    excess_all = ref_tree.query(exp)[0]
    stats = lambda d: {q: round(float(np.percentile(d, p)), 3) for q, p in (("p50", 50), ("p95", 95), ("p99", 99), ("max", 100))}
    out = {}
    for p in PARTS:
        outside = ~inside_any(ref[p], planes)
        protrusion = np.zeros(len(ref[p]))
        if outside.any():
            protrusion[outside] = bnd_tree.query(ref[p][outside])[0]
        mine = excess_all[owner == p]
        assert len(mine) > 0, f"part {p} shows no exposed surface"
        out[p] = {"pieces": len(pieces[p]), "ref_samples": int(len(ref[p])), "exposed_samples": int(len(mine)),
                  "outside_fraction": round(float(outside.mean()), 4), "protrusion_mm": stats(protrusion),
                  "excess_mm": stats(mine), "spacing_mm": SAMPLE_SPACING_MM, "interface_mm": INTERFACE_MM}
    return out


def score_build(out: Path, manifest: Dict[str, Any], v_mm: np.ndarray, groups: Sequence[Tuple[str, np.ndarray]]) -> None:
    """Fill each part's fidelity in the manifest from the whole-robot score of its built pieces."""
    pieces: Dict[str, List[Tuple[np.ndarray, np.ndarray, str]]] = {}
    for name in PARTS:
        frame = manifest["frames"][name]
        pieces[name] = []
        for item in manifest["parts"][name]["collision"]:
            pv, pg = read_obj(out / item["file"])
            pieces[name].append((to_robot_frame(pv, frame), pg[0][1], item["friction"]))
    scores = robot_fidelity(reference_surfaces(v_mm, groups), pieces)
    for name in PARTS:
        manifest["parts"][name]["fidelity"] = scores[name]

# ---- writing and loading a built body ---------------------------------------------------------

def write_obj(path: Path, verts: np.ndarray, tris: np.ndarray) -> str:
    """Write a triangle mesh in mm and return its sha256."""
    assert tris.min() >= 0 and tris.max() < len(verts), "triangle index out of range"
    lines = [f"v {x:.5f} {y:.5f} {z:.5f}" for x, y, z in verts]
    lines += [f"f {a + 1} {b + 1} {c + 1}" for a, b, c in tris]
    path.write_text("\n".join(lines) + "\n")
    return sha256_file(path)


def build(rig: Path, out: Path, v: Dict[str, Any], method: str) -> Dict[str, Any]:
    """Cut the rig into the model's parts, write meshes and manifest.json to out/, return the manifest."""
    assert method in ("coacd", "groups"), "method is coacd or groups"
    if sha256_file(rig) != RIG["sha256"]:
        raise RuntimeError("rig file does not match the pinned sha256")
    verts_cm, groups = read_obj(rig)
    reg = register(verts_cm, groups)
    v_mm = to_robot(verts_cm, reg)
    feats = measure(v_mm, groups)
    drift = check_against_table(v, feats)
    if drift:
        raise ValueError("rig disagrees with the parameter table: " + "; ".join(drift))
    frames = part_frames(v, feats)
    parts = split_parts(v_mm, groups, frames)
    out.mkdir(parents=True, exist_ok=True)
    manifest: Dict[str, Any] = {"rig": dict(RIG, url=RIG_URL), "registration": reg, "features": feats,
                                "frames": frames, "method": method, "coacd_threshold": COACD_THRESHOLD,
                                "tread_hull_clearance_mm": TREAD_HULL_CLEARANCE_MM, "parts": {}}
    for name in PARTS:
        manifest["parts"][name] = build_part(out, name, parts[name], method)
    score_build(out, manifest, v_mm, groups)
    (out / MANIFEST).write_text(json.dumps(manifest, indent=1) + "\n")
    return manifest


def rescore(out: Path, rig: Path) -> Dict[str, Any]:
    """Recompute the fidelity of an existing build (same rig) without decomposing again."""
    manifest = load(out)
    verts_cm, groups = read_obj(rig)
    score_build(out, manifest, to_robot(verts_cm, manifest["registration"]), groups)
    manifest.pop("dir")
    (out / MANIFEST).write_text(json.dumps(manifest, indent=1) + "\n")
    return manifest


def build_part(out: Path, name: str, part: Dict[str, Any], method: str) -> Dict[str, Any]:
    pieces = collision_pieces(name, part, method)
    assert pieces, f"part {name} has no collision pieces"
    entry: Dict[str, Any] = {"collision": [], "visual": []}
    for k, (pv, pt, friction) in enumerate(pieces):
        file = f"{name}_c{k:02d}.obj"
        entry["collision"].append({"file": file, "friction": friction, "sha256": write_obj(out / file, pv, pt),
                                   "vertices": int(len(pv))})
    vis_v, vis_t = merge(part["visual"])
    entry["visual"].append({"file": f"{name}_visual.obj", "sha256": write_obj(out / f"{name}_visual.obj", vis_v, vis_t),
                            "triangles": int(len(vis_t))})
    return entry


def load(out: Path) -> Dict[str, Any]:
    """A built body's manifest, every file present and unchanged, with its directory recorded."""
    manifest = json.loads((out / MANIFEST).read_text())
    assert manifest["rig"]["sha256"] == RIG["sha256"], "body was built from a different rig"
    assert set(manifest["parts"]) == set(PARTS), "body is missing a part"
    for name, entry in manifest["parts"].items():
        for item in entry["collision"] + entry["visual"]:
            if sha256_file(out / item["file"]) != item["sha256"]:
                raise RuntimeError(f"{name}: {item['file']} changed since the build")
    manifest["dir"] = str(out.resolve())
    return manifest


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("command", choices=("fetch", "build", "check", "rescore"))
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--rig", type=Path, default=None)
    parser.add_argument("--method", choices=("coacd", "groups"), default=None)
    try:
        args = parser.parse_args(argv)
    except SystemExit:
        return EXIT_USAGE
    if args.command == "fetch":
        print(fetch(args.out))
        return 0
    if args.command == "check":
        return report(load(args.out))
    rig = args.rig or args.out / RIG_FILE
    if not rig.is_file():
        print(f"rig not found at {rig}; run fetch first", file=sys.stderr)
        return EXIT_MISSING
    if args.command == "rescore":
        return report(rescore(args.out, rig))
    from . import params

    method = args.method or ("coacd" if _has_coacd() else "groups")
    try:
        manifest = build(rig, args.out, params.values(params.load()), method)
    except RuntimeError as error:
        print(error, file=sys.stderr)
        return EXIT_HASH
    except ValueError as error:
        print(error, file=sys.stderr)
        return EXIT_DRIFT
    return report(manifest)


def _has_coacd() -> bool:
    import importlib.util

    return importlib.util.find_spec("coacd") is not None


# The fidelity a built body must reach for `check` to pass, mm, fixed before any build was scored:
# at most 1 mm of the rig's surface missing from 99 % of it, and filled concavities within 2 mm over 95 %
# of the exposed collision surface (four voxels). A body that misses them is reported, not used blind.
LIMITS = {"protrusion_p99": 1.0, "excess_p95": 2.0}


def report(manifest: Dict[str, Any]) -> int:
    failed = 0
    for name, entry in manifest["parts"].items():
        f = entry["fidelity"]
        ok = f["protrusion_mm"]["p99"] <= LIMITS["protrusion_p99"] and f["excess_mm"]["p95"] <= LIMITS["excess_p95"]
        failed += 0 if ok else 1
        print(f"{name:10s} {f['pieces']:3d} pieces  protrusion p50/p99/max {f['protrusion_mm']['p50']:.2f}/"
              f"{f['protrusion_mm']['p99']:.2f}/{f['protrusion_mm']['max']:.2f}  excess p50/p95/max "
              f"{f['excess_mm']['p50']:.2f}/{f['excess_mm']['p95']:.2f}/{f['excess_mm']['max']:.2f}  "
              f"{'ok' if ok else 'OUTSIDE LIMITS'}")
    return EXIT_FAILED if failed else 0


if __name__ == "__main__":
    sys.exit(main())

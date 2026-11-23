"""Optional printed replacement parts: swap a stock part's geometry, mass and inertia for a printed one.

    xml = printed_parts.build_mjcf(v, parts=("fork:dauler", "treads:dauler-obar"))
    sim = printed_parts.vector_sim(parts=("rims:dauler-stock",))   # a robot.VectorSim

Off by default. With no parts the model is the stock one, byte for byte: apply() hands back the
string it was given. A part is a published, printable mesh (Thingiverse, Printables) that people
fit to Vector in place of the stock fork, rims or treads. Nothing from those meshes is in this
repository: they are fetched from their sources at run time, checked against the SHA-256 recorded
here, and cached (VECTOR_PRINT_PARTS, default ~/.cache/vector-simulacrum/print-parts). Each source
keeps its own licence; see PARTS and evidence/printed-parts/README.md.

What a part changes, and where each number comes from (part_report() returns them all):

- fork: the fork's contact shape becomes boxes built from the mesh (voxelised at
  fork_voxel_mm and merged; the stock proxies stop touching anything), and the fork body's mass,
  centre of mass and inertia become the mesh's, computed exactly from its closed surface, times a
  filament density and fill fraction (assumed; MATERIALS). The wrist joint and the arm are unchanged.
- rims: each wheel's mass and inertia become the printed rim's, and its rolling radius becomes the
  printed rim's outer radius plus the tread thickness.
- treads: the rolling radius becomes the rim radius plus the printed band's thickness (its largest
  radial extent, lugs included, since the lugs touch the table), the contact width becomes the
  band's width, and the loop's mass is shared out by path length: the arc round each wheel turns
  with that wheel, the two straight runs ride with the chassis. The stock treads' mass, which the
  table lumps into the chassis (total_mass_g), is taken back out at the same places.

Geometry is in millimetres and grams like the parameter table; the MJCF it edits is in metres and
kilograms. No MuJoCo import.
"""

from __future__ import annotations

import hashlib
import io
import json
import math
import os
import threading
import urllib.request
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from . import mjcf

MM, GRAM = mjcf.MM, mjcf.GRAM
KINDS = ("fork", "rims", "treads")
MAX_TRIANGLES = 2_000_000
MAX_BOXES = 400
FETCH_TIMEOUT_S = 120
PRINTABLES_API = "https://api.printables.com/graphql/"
# A fork registered by matching must overlap the reference at least this many times better in its best
# orientation than in any orientation that differs in x or z. Chosen, not fitted.
MATCH_MARGIN = 1.5

# Filament densities, g/cm^3. Assumed: typical values from filament makers' datasheets, not weighed.
MATERIALS: Dict[str, float] = {"PLA": 1.24, "PETG": 1.27, "TPU": 1.21, "resin": 1.15}
DEFAULT_MATERIAL = {"fork": ("PLA", 1.0), "rims": ("PLA", 1.0), "treads": ("TPU", 1.0)}

# The stock rims' diameters, front and rear, in mm. Measured (evidence/printed-parts/measure.py) on the
# two independent stock-shaped prints: Dauler's wheel_front_stock / wheel_back_stock (Thingiverse 3764077)
# 26.00 / 18.50, and 10ZZ's Front Wheel / Back Wheel (Printables 1777652) 25.98 / 18.50.
STOCK_RIM_DIAMETER_MM = (26.0, 18.5)
# The stock tread loop's mass, g. Assumed, unweighed: a band the table's tread_width_mm wide, as thick
# as the gap between the table's wheel radius and the stock rim (1.5 mm front and rear), along the loop's
# mid-line round both rims, in silicone rubber of 1.10 g/cm^3. part_report() states the figure it used.
STOCK_TREAD_DENSITY_G_CM3 = 1.10

# The fork mesh's frame to the wrist frame, mm: the print-bed registration that experiments/righting/
# printed_fork.py and web/src/game/printparts.js use for Dauler's forks7.stl (fitted there to the five
# physical cube-push replays). Wrist x is -bed z, wrist y is -bed x, wrist z is bed y; the offsets follow.
FORK_BED_OFFSET_MM = (-26.52, -48.01, 36.79)


def _archive(thing: str, title: str, sha: str) -> Dict[str, str]:
    return {"kind": "archive.org", "url": f"https://archive.org/download/thingiverse-{thing}/{title}_{thing}.zip",
            "sha256": sha}


# name -> source, licence, files (role -> (member or file id, sha256)), what it replaces.
PARTS: Dict[str, Dict[str, Any]] = {
    "fork:dauler": {
        "kind": "fork", "title": "Anki Vector Lifting Forks", "author": "Dauler (Steve M)",
        "page": "https://www.thingiverse.com/thing:3731746", "licence": "CC BY-NC 4.0",
        "source": _archive("3731746", "Anki_Vector_Lifting_Forks",
                           "0344d40dea0106b11f922af91d4f949abad4698338dcf86e1d3292c2d345a350"),
        "files": {"fork": ("files/forks7.stl", "c9dcd46e3c4b2c4b4aa896256109cd0b583a68adec432d615499c9c506dd281a")},
        "replaces": "the fork (lift crossbar and side hooks)", "registration": "bed",
    },
    "fork:10zz": {
        "kind": "fork", "title": "Anki Vector Shell (Lift Hook)", "author": "10ZZ",
        "page": "https://www.printables.com/model/1777652", "licence": "Open Community License v1.1",
        "source": {"kind": "printables", "print": "1777652"},
        "files": {"fork": ("7405072", "19b9b5eec7016ac43c3176b850ae501f4f2adac08dcdb7315e49ba5e5cf2e467")},
        "replaces": "the fork (lift hook)", "registration": "match:fork:dauler",
    },
    "rims:dauler-stock": {
        "kind": "rims", "title": "Anki Vector Wheels v2 (stock design)", "author": "Dauler (Steve M)",
        "page": "https://www.thingiverse.com/thing:3764077", "licence": "CC BY-NC 4.0",
        "source": _archive("3764077", "Anki_Vector_Wheels_v2",
                           "b041e299f4f225746aae14c8d6f53fe825edfd6f91205a563b5b3ef4f58c9944"),
        "files": {"front": ("files/wheel_front_stock.stl", "37afeacb4b811d02537eeca9c1090675a2188b7a9348ce4893636af79873fe88"),
                  "rear": ("files/wheel_back_stock.stl", "e0cf202676553ae621cbb32086747d5d47cd0bfa3671d29d86493926997400bf")},
        "replaces": "the outer wheels (rims) under the treads",
    },
    "rims:dauler-spoke": {
        "kind": "rims", "title": "Anki Vector Wheels v2 (spoke design)", "author": "Dauler (Steve M)",
        "page": "https://www.thingiverse.com/thing:3764077", "licence": "CC BY-NC 4.0",
        "source": _archive("3764077", "Anki_Vector_Wheels_v2",
                           "b041e299f4f225746aae14c8d6f53fe825edfd6f91205a563b5b3ef4f58c9944"),
        "files": {"front": ("files/wheel_front_spoke.stl", "4d0289ff8d0d3e3a356878c90c1c35e819e8cc622e7720a0ac3ef47a9ac25565"),
                  "rear": ("files/wheel_back_spoke.stl", "268ac509020c6a7180c05a6bf0165617503a76470c4b2f78c2a509e134249b7b")},
        "replaces": "the outer wheels (rims) under the treads",
    },
    "rims:10zz": {
        "kind": "rims", "title": "Anki Vector Shell (Front Wheel, Back Wheel)", "author": "10ZZ",
        "page": "https://www.printables.com/model/1777652", "licence": "Open Community License v1.1",
        "source": {"kind": "printables", "print": "1777652"},
        "files": {"front": ("7405100", "2d72557be9804d818396d5d1eff59b083c74f561a989e8b122581c3851842025"),
                  "rear": ("7405101", "f624564efccabb0dffb66d06e44cb2a1d5c58d4de599be556a4fd1e6429085c7")},
        "replaces": "the outer wheels (rims) under the treads",
    },
    "treads:dauler-obar": {
        "kind": "treads", "title": "Anki Vector improved treads (original bar)", "author": "Dauler (Steve M)",
        "page": "https://www.thingiverse.com/thing:3464746", "licence": "CC BY-NC 4.0",
        "source": _archive("3464746", "Anki_Vector_improved_treads",
                           "27d4310c7b75f8634612e00d9e957080da76b9dd50107a007d66060d86f2c342"),
        "files": {"left": ("files/tread_oBar.stl", "58094321241b215e9950a9e9a5a702447cabf61773075b867b249b756c292655"),
                  "right": ("files/tread_oBar.stl", "58094321241b215e9950a9e9a5a702447cabf61773075b867b249b756c292655")},
        "replaces": "the rubber treads",
    },
    "treads:dauler-diag": {
        "kind": "treads", "title": "Anki Vector improved treads (diagonal bar, directional)", "author": "Dauler (Steve M)",
        "page": "https://www.thingiverse.com/thing:3464746", "licence": "CC BY-NC 4.0",
        "source": _archive("3464746", "Anki_Vector_improved_treads",
                           "27d4310c7b75f8634612e00d9e957080da76b9dd50107a007d66060d86f2c342"),
        "files": {"left": ("files/tread_diag_left.stl", "7be0719cf5e4366ef46cdd41386732bd3a7406946fb8e970b0afcd32856db56e"),
                  "right": ("files/tread_diag_right.stl", "7ace84b59b8563d3cbd8634552a784136f7e84a533006f5da195439c2efed9c4")},
        "replaces": "the rubber treads",
    },
    "treads:dauler-sq-knobby": {
        "kind": "treads", "title": "Anki Vector improved treads (square knobby)", "author": "Dauler (Steve M)",
        "page": "https://www.thingiverse.com/thing:3464746", "licence": "CC BY-NC 4.0",
        "source": _archive("3464746", "Anki_Vector_improved_treads",
                           "27d4310c7b75f8634612e00d9e957080da76b9dd50107a007d66060d86f2c342"),
        "files": {"left": ("files/tread_sq_knobby.stl", "4557f44b01fd8d34838dc37079a527ddfd2068c021eaf345b359c7c92646487e"),
                  "right": ("files/tread_sq_knobby.stl", "4557f44b01fd8d34838dc37079a527ddfd2068c021eaf345b359c7c92646487e")},
        "replaces": "the rubber treads",
    },
    "treads:madizmo": {
        "kind": "treads", "title": "Anki Vector tracks (V2)", "author": "madizmo (Eric Beirnes)",
        "page": "https://www.thingiverse.com/thing:4287628", "licence": "CC BY 4.0",
        "source": _archive("4287628", "Anki_Vector_tracks",
                           "57f6ef8feb68e1507f0851b6fef89bcf263e84d09d6702b0dd8ef6548bea61f3"),
        "files": {"left": ("files/anki_vector_tracks_V2.stl", "898644b2bea1f108c45d88131171e9d8338f479ebef0280980261b1cc86635a9"),
                  "right": ("files/anki_vector_tracks_V2.stl", "898644b2bea1f108c45d88131171e9d8338f479ebef0280980261b1cc86635a9")},
        "replaces": "the rubber treads",
    },
}


# ---- meshes ---------------------------------------------------------------

def read_stl(data: bytes) -> np.ndarray:
    """Triangles (n, 3, 3) in the file's units from a binary or ASCII STL."""
    assert isinstance(data, (bytes, bytearray)) and len(data) >= 84, "not an STL"
    count = int.from_bytes(data[80:84], "little")
    if 0 < count <= MAX_TRIANGLES and len(data) == 84 + 50 * count:
        record = np.dtype([("normal", "<f4", 3), ("triangle", "<f4", (3, 3)), ("attribute", "<u2")])
        tris = np.frombuffer(bytes(data), offset=84, dtype=record, count=count)["triangle"].astype(float)
    else:
        words = bytes(data).decode("ascii", "strict").split()
        assert words[:1] == ["solid"], "STL is neither binary nor ASCII"
        coords = _ascii_vertices(words)
        assert len(coords) % 9 == 0 and 0 < len(coords) // 9 <= MAX_TRIANGLES, "ASCII STL is ragged or empty"
        tris = np.array(coords, float).reshape(-1, 3, 3)
    assert np.isfinite(tris).all(), "STL has non-finite coordinates"
    return tris


def _ascii_vertices(words: List[str]) -> List[float]:
    out: List[float] = []
    limit = len(words) - 3
    for i in range(limit):
        if words[i] == "vertex":
            out.extend(float(w) for w in words[i + 1:i + 4])
    return out


def mass_properties(tris_mm: np.ndarray, density_g_cm3: float, fill: float = 1.0) -> Dict[str, Any]:
    """Mass (g), centre of mass (mm) and inertia about it (g*mm^2, 3x3) of the solid a closed mesh bounds.

    Exact for a closed, consistently wound mesh: the sum over faces of signed tetrahedra from the origin."""
    assert tris_mm.ndim == 3 and tris_mm.shape[1:] == (3, 3) and len(tris_mm) >= 4, "need triangles"
    assert density_g_cm3 > 0.0 and 0.0 < fill <= 1.0, "density must be positive and fill inside (0, 1]"
    a, b, c = tris_mm[:, 0], tris_mm[:, 1], tris_mm[:, 2]
    det = np.einsum("ij,ij->i", a, np.cross(b, c))
    volume = det.sum() / 6.0
    assert volume > 0.0, "mesh encloses no volume (open, or wound inside out)"
    rho = density_g_cm3 * fill * 1e-3  # g/mm^3
    com = (det[:, None] * (a + b + c)).sum(0) / 24.0 / volume
    # The integral of x x^T over tetrahedron (0, a, b, c) is det/120 * A J A^T, A = [a b c], J = 1 + I.
    stacked = np.stack((a, b, c), axis=2)
    J = np.ones((3, 3)) + np.eye(3)
    second = np.einsum("n,nij,jk,nlk->il", det, stacked, J, stacked) / 120.0
    mass = rho * volume
    s_com = rho * second - mass * np.outer(com, com)
    inertia = np.trace(s_com) * np.eye(3) - s_com
    return {"volume_mm3": float(volume), "mass_g": float(mass), "com_mm": com, "inertia_gmm2": inertia}


def voxelise(tris_mm: np.ndarray, pitch_mm: float) -> Tuple[np.ndarray, np.ndarray]:
    """Occupancy grid (bool, nx ny nz) of the mesh's inside and the grid's corner (mm).

    A voxel is inside when a vertical ray through its column crosses the surface an odd number of
    times below its centre. Columns are nudged off the grid lines so no ray runs through a vertex."""
    assert pitch_mm > 0.0, "pitch must be positive"
    lo = tris_mm.reshape(-1, 3).min(0)
    hi = tris_mm.reshape(-1, 3).max(0)
    n = np.maximum(1, np.ceil((hi - lo) / pitch_mm).astype(int))
    assert int(np.prod(n)) <= 4_000_000, "grid too fine for this part"
    nudge = pitch_mm * np.array([1.234567e-4, 2.345678e-4])
    xs = lo[0] + (np.arange(n[0]) + 0.5) * pitch_mm + nudge[0]
    ys = lo[1] + (np.arange(n[1]) + 0.5) * pitch_mm + nudge[1]
    zs = lo[2] + (np.arange(n[2]) + 0.5) * pitch_mm
    occ = np.zeros(tuple(n), bool)
    tri_ylo, tri_yhi = tris_mm[:, :, 1].min(1), tris_mm[:, :, 1].max(1)
    for j, y in enumerate(ys):
        row = tris_mm[(tri_ylo <= y) & (tri_yhi >= y)]
        if len(row) == 0:
            continue
        occ[:, j, :] = _row_occupancy(row, xs, y, zs)
    return occ, lo


def _row_occupancy(row: np.ndarray, xs: np.ndarray, y: float, zs: np.ndarray) -> np.ndarray:
    """(nx, nz) inside test for the columns at (xs, y)."""
    p0, p1, p2 = row[:, 0], row[:, 1], row[:, 2]
    d1, d2 = p1 - p0, p2 - p0
    den = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
    keep = np.abs(den) > 1e-12
    p0, d1, d2, den = p0[keep], d1[keep], d2[keep], den[keep]
    px = xs[None, :] - p0[:, 0:1]
    py = y - p0[:, 1:2]
    u = (px * d2[:, 1:2] - py * d2[:, 0:1]) / den[:, None]
    w = (d1[:, 0:1] * py - d1[:, 1:2] * px) / den[:, None]
    hit = (u >= 0.0) & (w >= 0.0) & (u + w <= 1.0)
    z = p0[:, 2:3] + u * d1[:, 2:3] + w * d2[:, 2:3]
    out = np.zeros((len(xs), len(zs)), bool)
    for i in range(len(xs)):
        crossings = np.sort(z[hit[:, i], i])
        if len(crossings):
            out[i] = (np.searchsorted(crossings, zs) % 2) == 1
    return out


def boxes_from_grid(occ: np.ndarray, corner_mm: np.ndarray, pitch_mm: float) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Cover the occupied voxels exactly with boxes, greedily: grow along x, then y, then z.
    Returns (centre mm, half size mm) pairs. Deterministic for a given grid."""
    todo = occ.copy()
    nx, ny, nz = occ.shape
    boxes: List[Tuple[np.ndarray, np.ndarray]] = []
    for i, j, k in zip(*np.nonzero(occ)):
        if not todo[i, j, k]:
            continue
        i2 = i
        while i2 + 1 < nx and todo[i2 + 1, j, k]:
            i2 += 1
        j2 = j
        while j2 + 1 < ny and todo[i:i2 + 1, j2 + 1, k].all():
            j2 += 1
        k2 = k
        while k2 + 1 < nz and todo[i:i2 + 1, j:j2 + 1, k2 + 1].all():
            k2 += 1
        todo[i:i2 + 1, j:j2 + 1, k:k2 + 1] = False
        lo = corner_mm + np.array((i, j, k)) * pitch_mm
        hi = corner_mm + np.array((i2 + 1, j2 + 1, k2 + 1)) * pitch_mm
        boxes.append((0.5 * (lo + hi), 0.5 * (hi - lo)))
        assert len(boxes) <= MAX_BOXES, "part needs too many boxes; use a coarser fork_voxel_mm"
    assert not todo.any(), "boxes left voxels uncovered"
    return boxes


def ring(tris_mm: np.ndarray) -> Dict[str, float]:
    """A wheel or tread printed flat: its axis is its thinnest extent. Inner and outer radius, width (mm)."""
    pts = tris_mm.reshape(-1, 3)
    lo, hi = pts.min(0), pts.max(0)
    ext = hi - lo
    axis = int(np.argmin(ext))
    other = [i for i in range(3) if i != axis]
    centre = 0.5 * (lo + hi)
    assert abs(ext[other[0]] - ext[other[1]]) < 0.02 * ext[other[0]], "not round about its thinnest axis"
    r = np.hypot(pts[:, other[0]] - centre[other[0]], pts[:, other[1]] - centre[other[1]])
    return {"axis": axis, "inner_mm": float(r.min()), "outer_mm": float(r.max()), "width_mm": float(ext[axis])}


# ---- fetching -------------------------------------------------------------

def cache_dir() -> Path:
    env = os.environ.get("VECTOR_PRINT_PARTS")
    return Path(env) if env else Path.home() / ".cache" / "vector-simulacrum" / "print-parts"


def _cached(name: str, role: str) -> Path:
    return cache_dir() / name.replace(":", "_") / f"{role}.stl"


def load(name: str, fetch: bool = False) -> Dict[str, np.ndarray]:
    """The part's meshes by role, each checked against its SHA-256. fetch=True downloads what is missing."""
    assert name in PARTS, f"unknown part {name!r}; known: {sorted(PARTS)}"
    out: Dict[str, np.ndarray] = {}
    for role, (ref, sha) in PARTS[name]["files"].items():
        path = _cached(name, role)
        if not path.is_file():
            if not fetch:
                raise FileNotFoundError(f"{name} {role} is not cached at {path}; call fetch_part({name!r}) first")
            fetch_part(name)
        data = path.read_bytes()
        if hashlib.sha256(data).hexdigest() != sha:
            raise ValueError(f"{name} {role}: {path} is not the registered file (SHA-256 differs)")
        out[role] = read_stl(data)
    return out


def fetch_part(name: str) -> None:
    """Download a part's files from its source into the cache, keeping only files with the registered hash."""
    part = PARTS[name]
    source = part["source"]
    files: Dict[str, bytes] = {}
    if source["kind"] == "archive.org":
        blob = _get(source["url"])
        if hashlib.sha256(blob).hexdigest() != source["sha256"]:
            raise ValueError(f"{name}: archive at {source['url']} is not the registered one")
        with zipfile.ZipFile(io.BytesIO(blob)) as archive:
            for role, (member, _) in part["files"].items():
                files[role] = archive.read(member)
    else:
        assert source["kind"] == "printables", "unknown source kind"
        for role, (file_id, _) in part["files"].items():
            files[role] = _get(_printables_link(source["print"], file_id))
    for role, data in files.items():
        if hashlib.sha256(data).hexdigest() != part["files"][role][1]:
            raise ValueError(f"{name} {role}: downloaded file is not the registered one")
        path = _cached(name, role)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)


def _get(url: str, body: Optional[bytes] = None) -> bytes:
    request = urllib.request.Request(url, data=body, headers={"user-agent": "vector-simulacrum",
                                                               "content-type": "application/json"})
    with urllib.request.urlopen(request, timeout=FETCH_TIMEOUT_S) as response:
        return response.read()


def _printables_link(print_id: str, file_id: str) -> str:
    query = ('mutation { getDownloadLink(id: "%s", printId: "%s", fileType: stl, source: model_detail) '
             '{ ok output { link } } }') % (file_id, print_id)
    reply = json.loads(_get(PRINTABLES_API, json.dumps({"query": query}).encode()))
    link = (((reply.get("data") or {}).get("getDownloadLink") or {}).get("output") or {}).get("link")
    if not link:
        raise ValueError(f"Printables gave no download link for file {file_id}: {str(reply)[:200]}")
    return link


# ---- registration ---------------------------------------------------------

def fork_in_wrist(v: Dict[str, Any], tris: np.ndarray) -> np.ndarray:
    """forks7.stl's print-bed coordinates to the wrist frame (mm)."""
    ox, oy, oz = FORK_BED_OFFSET_MM
    x, y, z = tris[..., 0], tris[..., 1], tris[..., 2]
    return np.stack((-z + v["lift_front_wrt_wrist_mm"] + ox, -x + oy,
                     y + v["lift_xbar_top_wrt_wrist_mm"] - v["lift_xbar_height_mm"] + oz), axis=-1)


def match_fork(reference: np.ndarray, tris: np.ndarray, pitch_mm: float) -> Tuple[np.ndarray, float, float]:
    """Place another fork mesh (already in the wrist's axis order) like a registered one: each of the eight
    mirror choices about its box centre, centred on the reference's box, keeping the best overlap of their
    voxels. A fork is symmetric left to right, so a choice and its mirror in y score alike; the runner-up is
    the best choice that differs from the winner in x or z. Returns the placed mesh, the overlap (intersection
    over union) and the runner-up's overlap."""
    ref_lo, ref_hi = reference.reshape(-1, 3).min(0), reference.reshape(-1, 3).max(0)
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    scored: List[Tuple[float, int, np.ndarray]] = []
    for flips in range(8):
        sign = np.array([-1.0 if flips >> b & 1 else 1.0 for b in range(3)])
        placed = (tris - 0.5 * (lo + hi)) * sign + 0.5 * (ref_lo + ref_hi)
        if sign.prod() < 0.0:
            placed = placed[:, ::-1]  # a mirror turns the surface inside out; reverse each face back
        scored.append((_overlap(reference, placed, pitch_mm), flips, placed))
    best = max(scored, key=lambda s: (s[0], -s[1]))
    runner = max(s[0] for s in scored if (s[1] ^ best[1]) & 0b101)
    return best[2], best[0], runner


def _overlap(a: np.ndarray, b: np.ndarray, pitch_mm: float) -> float:
    lo = np.minimum(a.reshape(-1, 3).min(0), b.reshape(-1, 3).min(0)) - pitch_mm
    occ_a, ca = voxelise(a, pitch_mm)
    occ_b, cb = voxelise(b, pitch_mm)
    shape = tuple(np.ceil((np.maximum(a.reshape(-1, 3).max(0), b.reshape(-1, 3).max(0)) - lo) / pitch_mm).astype(int) + 2)
    grid_a, grid_b = np.zeros(shape, bool), np.zeros(shape, bool)
    for grid, occ, corner in ((grid_a, occ_a, ca), (grid_b, occ_b, cb)):
        off = np.round((corner - lo) / pitch_mm).astype(int)
        grid[off[0]:off[0] + occ.shape[0], off[1]:off[1] + occ.shape[1], off[2]:off[2] + occ.shape[2]] |= occ
    union = (grid_a | grid_b).sum()
    return float((grid_a & grid_b).sum() / union) if union else 0.0


def _axis_to_y(tris: np.ndarray, axis: int) -> np.ndarray:
    """Centre a round part on its box and turn its axis onto y."""
    pts = tris.reshape(-1, 3)
    centred = tris - 0.5 * (pts.min(0) + pts.max(0))
    order = {0: (1, 0, 2), 1: (0, 1, 2), 2: (0, 2, 1)}[axis]
    turned = centred[..., list(order)]
    return turned if axis == 1 else turned[:, ::-1]  # swapping two axes is a mirror; reverse each face back


# ---- the model ------------------------------------------------------------

def _material(kind: str, materials: Optional[Mapping[str, Tuple[str, float]]]) -> Tuple[str, float, float]:
    name, fill = (materials or {}).get(kind, DEFAULT_MATERIAL[kind])
    assert name in MATERIALS, f"unknown material {name!r}; known: {sorted(MATERIALS)}"
    assert 0.0 < fill <= 1.0, "fill must be inside (0, 1]"
    return name, MATERIALS[name], fill


def resolve(parts: Sequence[str]) -> Dict[str, str]:
    """kind -> part name; at most one part of each kind."""
    chosen: Dict[str, str] = {}
    for name in parts:
        assert name in PARTS, f"unknown part {name!r}; known: {sorted(PARTS)}"
        kind = PARTS[name]["kind"]
        assert kind not in chosen, f"two {kind} parts: {chosen[kind]} and {name}"
        chosen[kind] = name
    return chosen


def part_report(v: Dict[str, Any], parts: Sequence[str], materials: Optional[Mapping[str, Tuple[str, float]]] = None,
                fetch: bool = False, fork_voxel_mm: float = 1.0) -> Dict[str, Any]:
    """Every number apply() puts in the model, with where it came from."""
    chosen = resolve(parts)
    report: Dict[str, Any] = {}
    rim_r = (0.5 * STOCK_RIM_DIAMETER_MM[0], 0.5 * STOCK_RIM_DIAMETER_MM[1])
    if "fork" in chosen:
        report["fork"] = _fork_report(v, chosen["fork"], _material("fork", materials), fetch, fork_voxel_mm)
    if "rims" in chosen:
        report["rims"] = _rims_report(chosen["rims"], _material("rims", materials), fetch)
        rim_r = (report["rims"]["front"]["outer_mm"], report["rims"]["rear"]["outer_mm"])
    if "treads" in chosen:
        report["treads"] = _treads_report(v, chosen["treads"], _material("treads", materials), fetch, rim_r)
    report["stock_tread"] = _stock_tread(v)
    report["rolling_radius_mm"] = _rolling_radii(v, report, rim_r)
    return report


def _fork_report(v: Dict[str, Any], name: str, mat: Tuple[str, float, float], fetch: bool,
                 pitch: float) -> Dict[str, Any]:
    mesh = load(name, fetch)["fork"]
    overlap = runner = None
    if PARTS[name]["registration"] == "bed":
        placed = fork_in_wrist(v, mesh)
    else:
        reference = fork_in_wrist(v, load(PARTS[name]["registration"].split(":", 1)[1], fetch)["fork"])
        placed, overlap, runner = match_fork(reference, fork_in_wrist(v, mesh), pitch)
        # Two different designs overlap only partly; what must hold is that one orientation clearly wins.
        assert overlap >= MATCH_MARGIN * runner, f"{name}: no clear orientation ({overlap:.2f} against {runner:.2f})"
    props = mass_properties(placed, mat[1], mat[2])
    occ, corner = voxelise(placed, pitch)
    boxes = boxes_from_grid(occ, corner, pitch)
    return {"part": name, "material": mat[0], "density_g_cm3": mat[1], "fill": mat[2], **props,
            "boxes": boxes, "voxel_mm": pitch, "voxel_volume_mm3": float(occ.sum() * pitch ** 3),
            "registration_overlap": overlap, "registration_runner_up": runner, "replaces_mass_g": mjcf.derive(v)["masses_g"]["fork"]}


def _rims_report(name: str, mat: Tuple[str, float, float], fetch: bool) -> Dict[str, Any]:
    meshes = load(name, fetch)
    out: Dict[str, Any] = {"part": name, "material": mat[0], "density_g_cm3": mat[1], "fill": mat[2]}
    for end in ("front", "rear"):
        shape = ring(meshes[end])
        props = mass_properties(_axis_to_y(meshes[end], shape["axis"]), mat[1], mat[2])
        out[end] = {**shape, **props}
    return out


def _treads_report(v: Dict[str, Any], name: str, mat: Tuple[str, float, float], fetch: bool,
                   rim_r: Tuple[float, float]) -> Dict[str, Any]:
    meshes = load(name, fetch)
    out: Dict[str, Any] = {"part": name, "material": mat[0], "density_g_cm3": mat[1], "fill": mat[2]}
    for side in ("left", "right"):
        shape = ring(meshes[side])
        props = mass_properties(meshes[side], mat[1], mat[2])
        thickness = shape["outer_mm"] - shape["inner_mm"]
        out[side] = {**shape, "mass_g": props["mass_g"], "volume_mm3": props["volume_mm3"], "thickness_mm": thickness,
                     "share_g": _loop_shares(v, props["mass_g"], rim_r, thickness)}
    return out


def _loop_shares(v: Dict[str, Any], mass_g: float, rim_r: Tuple[float, float], thickness: float) -> Dict[str, float]:
    """A tread loop's mass split by path length along its mid-line: half round each wheel, two straight runs."""
    arcs = (math.pi * (rim_r[0] + 0.5 * thickness), math.pi * (rim_r[1] + 0.5 * thickness))
    runs = 2.0 * v["wheel_base_mm"]
    total = arcs[0] + arcs[1] + runs
    assert total > 0.0 and mass_g > 0.0, "loop must have length and mass"
    return {"front": mass_g * arcs[0] / total, "rear": mass_g * arcs[1] / total, "runs": mass_g * runs / total}


def _stock_tread(v: Dict[str, Any]) -> Dict[str, Any]:
    rim = (0.5 * STOCK_RIM_DIAMETER_MM[0], 0.5 * STOCK_RIM_DIAMETER_MM[1])
    thick = (v["wheel_front_radius_mm"] - rim[0], v["wheel_rear_radius_mm"] - rim[1])
    assert min(thick) > 0.0, "the table's wheels are not larger than the stock rims"
    t = 0.5 * (thick[0] + thick[1])
    length = math.pi * (rim[0] + rim[1] + t) + 2.0 * v["wheel_base_mm"]
    mass = length * v["tread_width_mm"] * t * STOCK_TREAD_DENSITY_G_CM3 * 1e-3
    return {"thickness_mm": thick, "mass_g": mass, "share_g": _loop_shares(v, mass, rim, t),
            "provenance": "assumed: unweighed; stock band geometry from the table and the measured rims, silicone 1.10 g/cm^3"}


def _rolling_radii(v: Dict[str, Any], report: Dict[str, Any], rim_r: Tuple[float, float]) -> Dict[str, Tuple[float, float]]:
    """(front, rear) rolling radius per side: rim plus whichever tread is fitted."""
    out: Dict[str, Tuple[float, float]] = {}
    stock = report["stock_tread"]["thickness_mm"]
    for side in ("left", "right"):
        thick = (report["treads"][side]["thickness_mm"],) * 2 if "treads" in report else stock
        out[side] = (rim_r[0] + thick[0], rim_r[1] + thick[1])
    return out


def apply(xml: str, v: Dict[str, Any], parts: Sequence[str] = (), materials: Optional[Mapping[str, Tuple[str, float]]] = None,
          fetch: bool = False, fork_voxel_mm: float = 1.0) -> str:
    """The model with the chosen printed parts in place of the stock ones. No parts: xml, unchanged."""
    if not parts:
        return xml
    report = part_report(v, parts, materials, fetch, fork_voxel_mm)
    root = ET.fromstring(xml)
    if any("_hull_" in g.get("name", "") for g in root.iter("geom")):
        _check_body_mode(root, report)
    if "fork" in report:
        _apply_fork(root, v, report["fork"])
    _apply_wheels(root, v, report)
    if "treads" in report:
        _apply_tread_runs(root, v, report)
    return ET.tostring(root, encoding="unicode") + "\n"


def _check_body_mode(root: ET.Element, report: Dict[str, Any]) -> None:
    """What may go on a body-mesh model (mjcf body=, whose contact comes from *_hull_* pieces of Anki's rig).

    The fork: nothing, since that mode builds its own fork as fork_hull_* pieces. Rims and treads: only if
    every rolling radius is at least the model's own wheel radius. That mode's tread hulls are fixed chassis
    pieces, cut 0.3 mm above the table and shrunk 0.5 mm inside the tip supports; a larger wheel moves the
    wheels and supports away from them, but a smaller one can set a hull on the table."""
    assert "fork" not in report, "a printed fork cannot go on a body-mesh model yet"
    if "rims" not in report and "treads" not in report:
        return
    for side, (front, rear) in report["rolling_radius_mm"].items():
        for name, radius in ((f"wheel_f{side[0]}", front), (f"wheel_r{side[0]}", rear)):
            geom = _body(root, name).find("geom")
            assert geom is not None, f"{name} has no geom"
            stock = float(geom.get("size")) / MM
            assert radius >= stock - 1e-6, \
                f"on a body-mesh model {name} would shrink from {stock:.3f} to {radius:.3f} mm and set the tread hull down"


def build_mjcf(v: Dict[str, Any], parts: Sequence[str] = (), obstacles: Sequence[Sequence[float]] = (),
               cube: Any = None, **kwargs: Any) -> str:
    base = mjcf.build_mjcf(v, obstacles) if cube is None else mjcf.build_mjcf(v, obstacles, cube)
    return apply(base, v, parts, **kwargs)


_SIM_LOCK = threading.Lock()


def vector_sim(parts: Sequence[str] = (), materials: Optional[Mapping[str, Tuple[str, float]]] = None,
               fetch: bool = False, **kwargs: Any) -> Any:
    """A robot.VectorSim with printed parts, without a keyword of its own in VectorSim.

    VectorSim builds its model through mjcf.build_mjcf. For the length of the constructor that name is
    pointed at a wrapper that applies the parts to the model it returns, then put back (under a lock, so
    two threads building at once cannot see each other's wrapper). With no parts this is VectorSim(**kwargs).
    A parts keyword on VectorSim itself waits for the body-mesh mode's keyword, so the two land together."""
    from .robot import VectorSim  # noqa: PLC0415  (robot imports MuJoCo; this module does not)

    if not parts:
        return VectorSim(**kwargs)
    resolve(parts)
    stock = mjcf.build_mjcf

    def with_parts(v: Dict[str, Any], obstacles: Sequence[Sequence[float]] = (), cube: Any = None) -> str:
        xml = stock(v, obstacles) if cube is None else stock(v, obstacles, cube)
        return apply(xml, v, parts, materials, fetch)

    with _SIM_LOCK:
        mjcf.build_mjcf = with_parts
        try:
            sim = VectorSim(**kwargs)
        finally:
            mjcf.build_mjcf = stock
    assert mjcf.build_mjcf is stock, "the stock builder was not put back"
    sim.parts = tuple(parts)
    return sim


def _body(root: ET.Element, name: str) -> ET.Element:
    for elem in root.iter("body"):
        if elem.get("name") == name:
            return elem
    raise AssertionError(f"no body {name!r} in the model")


def _floats(text: Optional[str]) -> List[float]:
    assert text, "attribute missing"
    return [float(x) for x in text.split()]


def _vec(values: Sequence[float]) -> str:
    return mjcf.vec([float(x) for x in values])


def _inertial(body: ET.Element, mass_g: float, com_mm: Sequence[float], inertia_gmm2: np.ndarray) -> None:
    """Replace whatever a body's mass came from with this inertial (first child, as MJCF wants it)."""
    for old in body.findall("inertial"):
        body.remove(old)
    full = inertia_gmm2 * GRAM * MM * MM
    attrs = {"pos": _vec([c * MM for c in com_mm]), "mass": mjcf.num(mass_g * GRAM),
             "fullinertia": _vec((full[0, 0], full[1, 1], full[2, 2], full[0, 1], full[0, 2], full[1, 2]))}
    joint_at = [i for i, child in enumerate(body) if child.tag in ("joint", "freejoint")]
    elem = ET.Element("inertial", attrs)
    body.insert(joint_at[-1] + 1 if joint_at else 0, elem)


def _apply_fork(root: ET.Element, v: Dict[str, Any], fork: Dict[str, Any]) -> None:
    body = _body(root, "lift_fork")
    reference = next(g for g in body.findall("geom") if g.get("name") == "lift_fork_geom")
    contact = {k: val for k, val in reference.attrib.items() if k not in ("name", "type", "pos", "size", "mass")}
    for geom in body.findall("geom"):
        if geom.get("name", "").startswith(("lift_fork_geom", "fork_side_", "fork_hook_", "fork_upright_")):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
            geom.set("mass", "0")
            geom.set("rgba", "0 0 0 0")
    _inertial(body, fork["mass_g"], fork["com_mm"], fork["inertia_gmm2"])
    for index, (centre, half) in enumerate(fork["boxes"]):
        ET.SubElement(body, "geom", dict(contact, name=f"printed_fork_{index}", type="box", mass="0",
                                         pos=_vec(centre * MM), size=_vec(half * MM)))


def _wheel_inertia(mass_g: float, radius_mm: float, width_mm: float) -> np.ndarray:
    """A solid cylinder about its centre, axis y (g*mm^2)."""
    axial = 0.5 * mass_g * radius_mm ** 2
    across = mass_g * (3.0 * radius_mm ** 2 + width_mm ** 2) / 12.0
    return np.diag((across, axial, across))


def _hoop_inertia(mass_g: float, radius_mm: float, width_mm: float) -> np.ndarray:
    """A thin hoop about its centre, axis y (g*mm^2): the half of a tread loop that wraps a wheel, as a hoop."""
    axial = mass_g * radius_mm ** 2
    across = 0.5 * mass_g * radius_mm ** 2 + mass_g * width_mm ** 2 / 12.0
    return np.diag((across, axial, across))


def _apply_wheels(root: ET.Element, v: Dict[str, Any], report: Dict[str, Any]) -> None:
    """Rolling radius, contact width and mass of the four wheels and the tip supports."""
    radii = report["rolling_radius_mm"]
    for name, side, end in (("wheel_fl", "left", 0), ("wheel_fr", "right", 0), ("wheel_rl", "left", 1), ("wheel_rr", "right", 1)):
        body = _body(root, name)
        geom = body.find("geom")
        assert geom is not None and geom.get("type") == "cylinder", f"{name} is not a cylinder"
        radius = radii[side][end]
        width = report["treads"][side]["width_mm"] if "treads" in report else v["tread_width_mm"]
        geom.set("size", mjcf.num(radius * MM))
        geom.set("fromto", _vec((0, -0.5 * width * MM, 0, 0, 0.5 * width * MM, 0)))
        key = ("front", "rear")[end]
        if "rims" in report:
            rim = report["rims"][key]
            mass, com, inertia = rim["mass_g"], rim["com_mm"], rim["inertia_gmm2"].copy()
        else:
            mass, com = v["wheel_mass_g"], np.zeros(3)
            inertia = _wheel_inertia(mass, radius, width)
        if "treads" in report:
            share = report["treads"][side]["share_g"][key]
            mass += share
            inertia = inertia + _hoop_inertia(share, radius - 0.5 * report["treads"][side]["thickness_mm"], width)
        geom.set("mass", "0")
        _inertial(body, mass, com, inertia)
    _apply_tip_supports(root, v, radii)
    _apply_tread_gear(root, radii)


def _apply_tread_gear(root: ET.Element, radii: Dict[str, Tuple[float, float]]) -> None:
    """A tread turns its two wheels at one surface speed (mjcf._equality_xml): the rear wheel's gear is the
    ratio of the rolling radii, which a printed rim or tread changes."""
    found = 0
    for eq in root.iter("joint"):
        name = eq.get("name")
        if eq.get("joint1") is None or name not in ("left_tread", "right_tread"):
            continue
        front, rear = radii["left" if name == "left_tread" else "right"]
        assert front > 0.0 and rear > 0.0, "rolling radii must be positive"
        eq.set("polycoef", f"0 {mjcf.num(front / rear)} 0 0 0")
        found += 1
    assert found == 2, "expected the two tread couplings"


def _apply_tip_supports(root: ET.Element, v: Dict[str, Any], radii: Dict[str, Tuple[float, float]]) -> None:
    """The body-fixed tipping cylinders (mjcf._tip_supports_xml) keep their clearance below the new wheels."""
    chassis = _body(root, "chassis")
    gap = v["tip_support_clearance_mm"]
    for geom in chassis.findall("geom"):
        name = geom.get("name", "")
        if not name.startswith("tip_"):
            continue
        end = 0 if name.startswith("tip_front") else 1
        side = "left" if name.endswith("_l") else "right"
        radius = radii[side][end]
        assert radius > gap, "tip support would have no size"
        geom.set("size", mjcf.num((radius - gap) * MM))


def _apply_tread_runs(root: ET.Element, v: Dict[str, Any], report: Dict[str, Any]) -> None:
    """Move the stock treads' mass out of the chassis and the printed loops' straight runs into it.

    The stock loop, lumped in the chassis by total_mass_g, comes out entirely (its arcs and its runs);
    the printed runs go back in. Arcs sit at the axles and runs midway between them, where the wheel bodies
    are in the model, so a refit that moves the wheels moves these too."""
    chassis = _body(root, "chassis")
    inertial = chassis.find("inertial")
    assert inertial is not None, "chassis has no inertial to adjust"
    stock = report["stock_tread"]["share_g"]
    points: List[Tuple[float, Tuple[float, float, float]]] = []
    for side, front_name, rear_name in (("left", "wheel_fl", "wheel_rl"), ("right", "wheel_fr", "wheel_rr")):
        # Where the axles are, read from the wheel bodies themselves (chassis frame, mm), not from the table.
        front = np.array(_floats(_body(root, front_name).get("pos"))) / MM
        rear = np.array(_floats(_body(root, rear_name).get("pos"))) / MM
        mid = tuple(0.5 * (front + rear))
        points += [(-stock["front"], tuple(front)), (-stock["rear"], tuple(rear)), (-stock["runs"], mid),
                   (report["treads"][side]["share_g"]["runs"], mid)]
    assert len(points) == 8, "two sides, four masses each"
    _add_point_masses(inertial, points)


def _add_point_masses(inertial: ET.Element, points: Sequence[Tuple[float, Tuple[float, float, float]]]) -> None:
    """Add (or with a negative mass remove) point masses to a diagonal inertial, keeping it diagonal.

    The pairs this is called with are mirror images in y, so the products of inertia they add cancel;
    xz products from a point off the centre of mass are left out, which the report states."""
    mass = float(inertial.get("mass")) / GRAM
    com = np.array([float(c) for c in inertial.get("pos").split()]) / MM
    diag = np.array([float(c) for c in inertial.get("diaginertia").split()]) / (GRAM * MM * MM)
    inertia_origin = diag + mass * np.array([com[1] ** 2 + com[2] ** 2, com[0] ** 2 + com[2] ** 2, com[0] ** 2 + com[1] ** 2])
    moment = mass * com
    for m, p in points:
        x, y, z = p
        mass += m
        moment = moment + m * np.array(p)
        inertia_origin = inertia_origin + m * np.array([y * y + z * z, x * x + z * z, x * x + y * y])
    assert mass > 0.0, "chassis mass went negative"
    com = moment / mass
    diag = inertia_origin - mass * np.array([com[1] ** 2 + com[2] ** 2, com[0] ** 2 + com[2] ** 2, com[0] ** 2 + com[1] ** 2])
    assert (diag > 0.0).all(), "chassis inertia went non-positive"
    inertial.set("mass", mjcf.num(mass * GRAM))
    inertial.set("pos", _vec(com * MM))
    inertial.set("diaginertia", _vec(diag * GRAM * MM * MM))

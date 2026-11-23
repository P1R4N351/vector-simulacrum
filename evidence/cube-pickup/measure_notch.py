"""Measure the lift pockets in the light cube's vertical edges from the animators' rig cube.

    python evidence/cube-pickup/measure_notch.py [--cache DIR] [--out FILE]

The cube's four vertical edges are notched over their middle (Vector TRM section 15: "The cube has
holes near the corners to allow the lift to engage"). Above and below the opening, each notch is
not a solid ledge but a thin lip on each of the two faces, closing a hollow pocket behind it. The
fork's barbed hook goes in through the opening; as the lift rises the lip drops into the barb's
valley and the barb's tip rises into the pocket. The SDK's cube.obj, from which
simulacrum/cube_surface.csv is built, is a 128-vertex shape that fills the pocket in.

This script fetches Victor_cube_rig.obj at the commit and sha256 web/tools/fetch_cube_model.py pins,
puts it in that tool's frame (z up, centred, 44 mm on each axis, the frame of cube_surface.csv),
samples its surface, and measures, for each of the four vertical edges at both ends and for the lip
on each of its two faces (16 cases, mirrored into one octant):

- lip bottom: the underside of the lip, just inside the face;
- lip inner face: where, going in from the face, the underside rises into the pocket;
- pocket ceiling: the underside just behind the lip;
- back wall: the first material behind the opening, at the opening's mid-height.

The cases must agree within AGREE_MM. Written for the model (mjcf.py carves the pocket from the
cube's boxes): the back wall rounded down, the lip's inner face rounded up and the ceiling rounded
up, each to 0.1 mm, so the carved pocket contains the measured one.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import sys
import urllib.request
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SPACING_MM = 0.1
BAND_MM = (15.5, 19.5)        # across the notch, clear of the other face's lip and the back wall
LIP_PROBE_MM = (21.0, 21.6)   # columns inside the lip
UNDERSIDE_ABOVE_MM = 4.0
RISE_MM = 2.0                 # an underside this far above the lip's is the pocket's
COLUMN_MM = 0.1
OPENING_Z_MM = 2.0
AGREE_MM = 0.3
TIMEOUT_S = 120
DEFAULT_CACHE = Path.home() / ".cache" / "vector-simulacrum-rig"


def fetch_tool():
    path = ROOT / "web" / "tools" / "fetch_cube_model.py"
    spec = importlib.util.spec_from_file_location("fetch_cube_model", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def rig_cube(cache: Path, tool) -> Path:
    cache.mkdir(parents=True, exist_ok=True)
    dest = cache / "Victor_cube_rig.obj"
    if not dest.is_file() or hashlib.sha256(dest.read_bytes()).hexdigest() != tool.CUBE_META["sha256"]:
        with urllib.request.urlopen(tool.CUBE_URL, timeout=TIMEOUT_S) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != tool.CUBE_META["sha256"]:
            raise RuntimeError("downloaded rig cube does not match the pinned sha256")
        dest.write_bytes(data)
    return dest


def samples(path: Path, tool) -> np.ndarray:
    verts, buckets = tool.parse_cube_obj(path.read_text())
    v = np.array(tool.to_canonical_vertices(verts)) * 1000.0
    tris = np.array([t for group in buckets.values() for t in group])
    a, b, c = v[tris[:, 0]], v[tris[:, 1]], v[tris[:, 2]]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    counts = np.maximum(1, np.ceil(area / SPACING_MM ** 2)).astype(np.int64)
    assert counts.sum() < 5_000_000, "too many samples; raise the spacing"
    owner = np.repeat(np.arange(len(tris)), counts)
    rng = np.random.default_rng(0)
    u, w = rng.random(len(owner)), rng.random(len(owner))
    flip = u + w > 1.0
    u[flip], w[flip] = 1.0 - u[flip], 1.0 - w[flip]
    return a[owner] + u[:, None] * (b - a)[owner] + w[:, None] * (c - a)[owner]


def case(p: np.ndarray) -> dict:
    """One lip, with the edge mirrored to +x +y +z and the lip on the +x face."""
    band = p[(p[:, 1] >= BAND_MM[0]) & (p[:, 1] <= BAND_MM[1])]
    above = band[band[:, 2] > UNDERSIDE_ABOVE_MM]
    probe = above[(above[:, 0] >= LIP_PROBE_MM[0]) & (above[:, 0] <= LIP_PROBE_MM[1])]
    assert len(probe) > 10, "no lip where one is expected"
    lip_bottom = float(probe[:, 2].min())
    inner, ceiling = None, None
    x = LIP_PROBE_MM[1]
    for _ in range(200):
        x -= COLUMN_MM
        cell = above[(above[:, 0] >= x) & (above[:, 0] < x + COLUMN_MM)]
        if len(cell) and float(cell[:, 2].min()) > lip_bottom + RISE_MM:
            inner = x + COLUMN_MM
            behind = above[(above[:, 0] >= inner - 1.5) & (above[:, 0] < inner - 0.3)]
            ceiling = float(np.median([behind[(behind[:, 0] >= c) & (behind[:, 0] < c + COLUMN_MM)][:, 2].min()
                                       for c in np.arange(inner - 1.5, inner - 0.3, COLUMN_MM)
                                       if len(behind[(behind[:, 0] >= c) & (behind[:, 0] < c + COLUMN_MM)])]))
            break
    assert inner is not None, "the underside never rose into a pocket"
    opening = band[(np.abs(band[:, 2]) < OPENING_Z_MM) & (band[:, 0] < inner)]
    assert len(opening) > 10, "no back wall behind the opening"
    back = float(opening[:, 0].max())
    return {"lip_bottom_mm": round(lip_bottom, 3), "lip_inner_mm": round(inner, 3),
            "ceiling_mm": round(ceiling, 3), "back_mm": round(back, 3)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    ap.add_argument("--out", type=Path, default=Path(__file__).with_name("notch.json"))
    args = ap.parse_args(argv)
    tool = fetch_tool()
    pts = samples(rig_cube(args.cache, tool), tool)
    cases = {}
    for sx in (1, -1):
        for sy in (1, -1):
            for sz in (1, -1):
                m = pts * np.array([sx, sy, sz])
                cases[f"x{sx:+d} y{sy:+d} z{sz:+d}, lip on x face"] = case(m)
                cases[f"x{sx:+d} y{sy:+d} z{sz:+d}, lip on y face"] = case(m[:, [1, 0, 2]])
    keys = ("lip_bottom_mm", "lip_inner_mm", "ceiling_mm", "back_mm")
    spread = {k: round(max(c[k] for c in cases.values()) - min(c[k] for c in cases.values()), 3) for k in keys}
    for k, s in spread.items():
        assert s <= AGREE_MM, f"the 16 cases disagree on {k} by {s} mm"
    back = math.floor(min(c["back_mm"] for c in cases.values()) * 10) / 10
    inner = math.ceil(max(c["lip_inner_mm"] for c in cases.values()) * 10) / 10
    ceiling = math.ceil(max(c["ceiling_mm"] for c in cases.values()) * 10) / 10
    result = {"source": {"url": tool.CUBE_URL, "sha256": tool.CUBE_META["sha256"]},
              "frame": "fetch_cube_model.py canonical: z up, centred, 44 mm on each axis; mm",
              "samples": int(len(pts)), "spacing_mm": SPACING_MM, "cases": cases, "spread_mm": spread,
              "pocket_xy_mm": [back, inner], "pocket_half_z_mm": ceiling,
              "lip_bottom_mm": round(float(np.mean([c["lip_bottom_mm"] for c in cases.values()])), 2)}
    args.out.write_text(json.dumps(result, indent=1) + "\n")
    for name, c in cases.items():
        print(name, c)
    print("spread", spread)
    print("pocket |x|,|y| in", result["pocket_xy_mm"], "|z| up to", ceiling, "lip bottom", result["lip_bottom_mm"])
    return 0


if __name__ == "__main__":
    sys.exit(main())

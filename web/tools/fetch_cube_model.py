#!/usr/bin/env python3
"""Fetch and normalize kercre123/vector-animations-raw Victor_cube_rig.obj into public/robot/sdk/."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from typing import Dict, List, Optional, Sequence, Tuple
import urllib.error
import urllib.request

HERE = Path(__file__).resolve().parent.parent
DEST = HERE / "public" / "robot" / "sdk"
CUBE_META = {
    "repo": "kercre123/vector-animations-raw",
    "commit": "59b198d4e6fb7394ae4c3d832dea2c4d28866ab0",
    "path": "assets/models/Vector/obj/Victor_cube_rig.obj",
    "sha256": "32a28d933e089661f34fadf63d3cc3df33b4c99a6e6b9c1b1394c2e24bad72df",
}
CUBE_URL = f"https://raw.githubusercontent.com/{CUBE_META['repo']}/{CUBE_META['commit']}/{CUBE_META['path']}"
DDL_URL = f"https://raw.githubusercontent.com/{CUBE_META['repo']}/main/LICENSE"
DDL_SHA256 = "6573b8f79ac25f8626b795b3322aa7420e87e21b0520d20ad90ebca7ba4e06f0"
TARGET_SIZE_M = 0.044
RECESSED_FACE_HALF_MM = 21.52
DOWNLOAD_TIMEOUT_S = 120
MAX_OBJ_LINES = 200_000


def sha256_bytes(data: bytes) -> str:
    """Return lowercase hex SHA-256 digest of bytes."""
    assert isinstance(data, (bytes, bytearray)) and len(data) > 0, "non-empty bytes required"
    return hashlib.sha256(data).hexdigest()


def ensure_ddl_license(ddl_license: Optional[Path]) -> None:
    """Ensure DDL-ASSET-LICENSE.txt exists in DEST with matching SHA-256."""
    assert DEST.name == "sdk", "unexpected destination root"
    target = DEST / "DDL-ASSET-LICENSE.txt"
    if target.is_file() and sha256_bytes(target.read_bytes()) == DDL_SHA256:
        return
    if ddl_license is not None:
        assert ddl_license.is_file(), f"missing DDL license file: {ddl_license}"
        data = ddl_license.read_bytes()
    else:
        with urllib.request.urlopen(DDL_URL, timeout=DOWNLOAD_TIMEOUT_S) as resp:
            data = resp.read()
    assert sha256_bytes(data) == DDL_SHA256, "DDL-ASSET-LICENSE.txt SHA-256 mismatch"
    DEST.mkdir(parents=True, exist_ok=True)
    target.write_bytes(data)


def parse_cube_obj(raw_text: str) -> Tuple[List[Tuple[float, float, float]], Dict[str, List[Tuple[int, int, int]]]]:
    """Parse vertices and fan-triangulate faces grouped by usemtl tag."""
    lines = raw_text.splitlines()
    assert 0 < len(lines) <= MAX_OBJ_LINES, "unexpected Victor_cube_rig line count"
    verts: List[Tuple[float, float, float]] = []
    buckets: Dict[str, List[Tuple[int, int, int]]] = {}
    cur_tag = "phong1SG"
    for raw in lines:
        parts = raw.split()
        if not parts:
            continue
        if parts[0] == "v" and len(parts) >= 4:
            verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
        elif parts[0] == "usemtl" and len(parts) >= 2:
            cur_tag = parts[1]
        elif parts[0] == "f" and len(parts) >= 4:
            ids = [int(w.split("/")[0]) - 1 for w in parts[1:]]
            tris = buckets.setdefault(cur_tag, [])
            for k in range(1, len(ids) - 1):
                tris.append((ids[0], ids[k], ids[k + 1]))
    assert verts and buckets, "empty cube geometry"
    return verts, buckets


def to_canonical_vertices(verts: Sequence[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
    """Transform Maya (X, Y, Z) to cyclic Z-up (Z, X, Y) centered and scaled to 44 mm."""
    assert len(verts) > 0, "vertices required"
    mins = [min(v[i] for v in verts) for i in range(3)]
    maxs = [max(v[i] for v in verts) for i in range(3)]
    spans = [maxs[i] - mins[i] for i in range(3)]
    assert all(s > 0.1 for s in spans), "degenerate cube span"
    mids = [0.5 * (mins[i] + maxs[i]) for i in range(3)]
    # Cyclic mapping: x = Z_maya, y = X_maya, z = Y_maya (det = +1).
    out: List[Tuple[float, float, float]] = []
    for xm, ym, zm in verts:
        x = (zm - mids[2]) * (TARGET_SIZE_M / spans[2])
        y = (xm - mids[0]) * (TARGET_SIZE_M / spans[0])
        z = (ym - mids[1]) * (TARGET_SIZE_M / spans[1])
        out.append((x, y, z))
    return out


def format_normalized_obj(
    verts: Sequence[Tuple[float, float, float]],
    buckets: Dict[str, List[Tuple[int, int, int]]],
) -> Tuple[str, int]:
    """Emit canonical single-material OBJ groups with 1-based vertex indices."""
    assert verts and buckets, "missing cube data"
    out = [f"# Normalized {CUBE_META['path']}@{CUBE_META['commit'][:12]}"]
    out.extend(f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in verts)
    tri_count = 0
    for tag, tris in buckets.items():
        if not tris:
            continue
        out.append(f"o cube_{tag}")
        out.append(f"usemtl {tag}")
        out.extend(f"f {a + 1} {b + 1} {c + 1}" for a, b, c in tris)
        tri_count += len(tris)
    assert tri_count > 0, "no triangles written"
    return "\n".join(out) + "\n", tri_count


def build_cube_assets(cube_path: Path, ddl_license: Optional[Path]) -> int:
    """Validate, normalize, and write Victor_cube_rig.obj and cube_rig_manifest.json."""
    if not cube_path.is_file():
        print(f"cube OBJ not found: {cube_path}", file=sys.stderr)
        return 3
    raw_bytes = cube_path.read_bytes()
    if sha256_bytes(raw_bytes) != CUBE_META["sha256"]:
        print("SHA-256 mismatch for Victor_cube_rig.obj", file=sys.stderr)
        return 4
    ensure_ddl_license(ddl_license)
    raw_verts, buckets = parse_cube_obj(raw_bytes.decode("utf-8", "replace"))
    norm_text, tri_count = format_normalized_obj(to_canonical_vertices(raw_verts), buckets)
    DEST.mkdir(parents=True, exist_ok=True)
    obj_path = DEST / "Victor_cube_rig.obj"
    obj_bytes = norm_text.encode("utf-8")
    obj_path.write_bytes(obj_bytes)
    manifest = {
        "source": f"{CUBE_META['repo']} {Path(CUBE_META['path']).name}@{CUBE_META['commit'][:12]}",
        "license": "Digital Dream Labs Software Asset License v1.0 (DDL-ASSET-LICENSE.txt)",
        "raw_sha256": CUBE_META["sha256"],
        "sha256": sha256_bytes(obj_bytes),
        "triangles": tri_count,
        "recessed_face_half_mm": RECESSED_FACE_HALF_MM,
    }
    (DEST / "cube_rig_manifest.json").write_text(json.dumps(manifest, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote 2 cube rig files to {DEST.relative_to(HERE)} ({tri_count} triangles)")
    return 0


def remove_cube_assets() -> int:
    """Remove generated Victor_cube_rig.obj and cube_rig_manifest.json."""
    assert DEST.name == "sdk", "unexpected destination root"
    removed = 0
    for name in ("Victor_cube_rig.obj", "cube_rig_manifest.json"):
        p = DEST / name
        if p.is_file():
            p.unlink()
            removed += 1
    assert removed >= 0, "invalid count"
    print(f"removed {removed} cube rig files")
    return 0


def main(argv: Sequence[str]) -> int:
    """CLI entrypoint for fetching or removing Victor_cube_rig.obj assets."""
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--cube", type=Path, help="Local path to raw Victor_cube_rig.obj")
    mode.add_argument("--download", action="store_true", help="Download raw Victor_cube_rig.obj from GitHub")
    mode.add_argument("--remove", action="store_true", help="Remove Victor_cube_rig.obj and cube_rig_manifest.json")
    p.add_argument("--ddl-license", type=Path, default=None, help="Optional local DDL-ASSET-LICENSE.txt")
    args = p.parse_args(list(argv))
    if args.remove:
        return remove_cube_assets()
    if args.download:
        with tempfile.TemporaryDirectory(prefix="vector-cube-") as td:
            tmp = Path(td) / "Victor_cube_rig.obj"
            try:
                with urllib.request.urlopen(CUBE_URL, timeout=DOWNLOAD_TIMEOUT_S) as resp:
                    tmp.write_bytes(resp.read())
            except (OSError, urllib.error.URLError) as err:
                print(f"cube download failed: {err}", file=sys.stderr)
                return 5
            return build_cube_assets(tmp, args.ddl_license)
    assert args.cube is not None, "--cube required"
    return build_cube_assets(args.cube, args.ddl_license)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

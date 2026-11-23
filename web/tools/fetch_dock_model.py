#!/usr/bin/env python3
"""Fetch Vector charger OBJ/MTL models from kercre123/vector-animations-raw into public/robot/sdk/.

The charger meshes live under the Digital Dream Labs Software Asset License
(DDL-ASSET-LICENSE.txt), which allows personal/commercial use with Vector but is
not an open-source license, so this repository does not check them in.

  python3 tools/fetch_dock_model.py              download upstream pinned files
  python3 tools/fetch_dock_model.py --assets DIR copy from a local Vector/obj folder
  python3 tools/fetch_dock_model.py --remove     delete the copied dock files

Exit: 0 done, 2 bad arguments, 3 assets missing, 4 hash mismatch, 5 download failed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Dict, Optional, Sequence

HERE = Path(__file__).resolve().parent.parent
DEST = HERE / "public" / "robot" / "sdk"
OBJ_BASE_URL = "https://raw.githubusercontent.com/kercre123/vector-animations-raw/main/assets/models/Vector/obj"
LICENSE_URL = "https://raw.githubusercontent.com/kercre123/vector-animations-raw/main/LICENSE"
DOWNLOAD_TIMEOUT_S = 30
MAX_FILES = 16

PINNED_URLS = {
    "Vic_charger_hp_geo.obj": f"{OBJ_BASE_URL}/Vic_charger_hp_geo.obj",
    "Vic_charger_hp_geo.mtl": f"{OBJ_BASE_URL}/Vic_charger_hp_geo.mtl",
    "vic_charger_midPoly.obj": f"{OBJ_BASE_URL}/vic_charger_midPoly.obj",
    "vic_charger_midPoly.mtl": f"{OBJ_BASE_URL}/vic_charger_midPoly.mtl",
    "DDL-ASSET-LICENSE.txt": LICENSE_URL,
}

PINNED_SHA256 = {
    "Vic_charger_hp_geo.obj": "2ccfb20d5ed4b793862f8deea940632a2f4f34a0dfc76f8fd6083e53419b3bb0",
    "Vic_charger_hp_geo.mtl": "673f08f255c6362cf160957b1a133486f03e47993fcacb55d57f16ac15a8539a",
    "vic_charger_midPoly.obj": "77ecb877f94cac0b25315ba99551eb105089dc7db7a8b793d1027d885f20ad4e",
    "vic_charger_midPoly.mtl": "dc12b76bf76f79ea7f906fd368a6459fe8828b51b07a78d8626ed2a4ba767442",
    "DDL-ASSET-LICENSE.txt": "6573b8f79ac25f8626b795b3322aa7420e87e21b0520d20ad90ebca7ba4e06f0",
}

CHARGER_FRAME = {
    "unit_m": 0.001,
    "lip_z_maya_mm": -81.2401,
    "bottom_y_maya_mm": -0.5586,
    "marker_wall_x_mm": 93.205,
    "marker_z_mm": 48.5,
    "marker_size_mm": 46.0,
    "pin_left_mm": [59.710, -9.850, 13.671],
    "pin_right_mm": [59.710, 9.850, 13.671],
}


def download_files() -> Optional[Dict[str, bytes]]:
    """Download all pinned charger OBJ, MTL, and license files from upstream."""
    assert len(PINNED_URLS) <= MAX_FILES, "too many dock files"
    out: Dict[str, bytes] = {}
    for name, url in PINNED_URLS.items():
        try:
            with urllib.request.urlopen(url, timeout=DOWNLOAD_TIMEOUT_S) as resp:
                data = resp.read()
        except (OSError, urllib.error.URLError) as err:
            print(f"download failed for {name}: {err}", file=sys.stderr)
            return None
        if not data:
            print(f"empty download for {name}", file=sys.stderr)
            return None
        out[name] = data
    assert len(out) == len(PINNED_URLS), "download set mismatch"
    return out


def read_folder(folder: Path) -> Optional[Dict[str, bytes]]:
    """Read required charger OBJ/MTL files and optional license from a local folder."""
    required = [n for n in PINNED_URLS if n != "DDL-ASSET-LICENSE.txt"]
    if not all((folder / n).is_file() for n in required):
        return None
    out = {n: (folder / n).read_bytes() for n in required}
    lic_candidates = (folder / "DDL-ASSET-LICENSE.txt", folder.parents[2] / "LICENSE" if len(folder.parents) >= 3 else None)
    for cand in lic_candidates:
        if cand is not None and cand.is_file():
            out["DDL-ASSET-LICENSE.txt"] = cand.read_bytes()
            break
    return out


def check_hashes(files: Dict[str, bytes]) -> Sequence[str]:
    """Return file names whose SHA-256 differs from the pinned upstream digest."""
    return [n for n, d in files.items() if n in PINNED_SHA256 and hashlib.sha256(d).hexdigest() != PINNED_SHA256[n]]


def write_copy(files: Dict[str, bytes], source: str) -> None:
    """Write fetched charger files and dock_manifest.json into public/robot/sdk/."""
    DEST.mkdir(parents=True, exist_ok=True)
    for name, data in files.items():
        (DEST / name).write_bytes(data)
    manifest = {
        "source": source,
        "sha256": {n: hashlib.sha256(d).hexdigest() for n, d in files.items()},
        "license": "Digital Dream Labs Software Asset License v1.0 (DDL-ASSET-LICENSE.txt)",
        "frame": CHARGER_FRAME,
    }
    (DEST / "dock_manifest.json").write_text(json.dumps(manifest, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {len(files) + 1} dock files to {DEST.relative_to(HERE)} (from {source})")


def remove_copy() -> int:
    """Delete downloaded dock assets and dock_manifest.json."""
    removed = 0
    for name in [*PINNED_URLS, "dock_manifest.json"]:
        path = DEST / name
        if path.is_file():
            path.unlink()
            removed += 1
    print(f"removed {removed} dock files")
    return 0


def main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--assets", type=Path, help="folder holding Vic_charger_hp_geo.obj and friends")
    group.add_argument("--remove", action="store_true", help="delete dock files in public/robot/sdk")
    parser.add_argument("--accept-other-version", action="store_true", help="copy even if SHA-256 hashes differ")
    args = parser.parse_args(list(argv))
    if args.remove:
        return remove_copy()
    if args.assets is not None:
        files = read_folder(args.assets)
        source = f"folder {args.assets.name}"
        if files is None:
            print(f"dock OBJ files not found in {args.assets}", file=sys.stderr)
            return 3
    else:
        files = download_files()
        source = "kercre123/vector-animations-raw main/assets/models/Vector/obj"
        if files is None:
            return 5
    changed = check_hashes(files)
    if changed and not args.accept_other_version:
        print(f"hash differs for: {', '.join(changed)} (pass --accept-other-version to copy anyway)", file=sys.stderr)
        return 4
    write_copy(files, source + (" (hashes differ from pinned)" if changed else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

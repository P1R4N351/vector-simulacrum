#!/usr/bin/env python3
"""Copy the Vector SDK's own 3D robot model into this app..

The page draws the robot with the model that the Vector Python SDK ships for
its 3D viewer (anki_vector/opengl/assets/vector.obj). That model is NOT part
of this repository: it is under the Anki Image and 3D Model License, which
allows use with the SDK and the robot but is not an open-source licence. This
tool copies it from an SDK you have, into public/robot/sdk/ (git-ignored).
Without it the page draws the physics model's own boxes and cylinders.

  python3 tools/fetch_sdk_model.py              from an installed anki_vector
  python3 tools/fetch_sdk_model.py --assets DIR from a folder holding vector.obj
  python3 tools/fetch_sdk_model.py --download   fetch the SDK wheel from PyPI
                                                (downloaded, not installed)
  python3 tools/fetch_sdk_model.py --remove     delete the copy

The three files are checked against the hashes of wirepod_vector_sdk 0.8.1.
Exit: 0 done, 2 bad arguments, 3 model not found, 4 hash mismatch, 5 download failed.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path
from typing import Dict, Optional, Sequence

HERE = Path(__file__).resolve().parent.parent
DEST = HERE / "public" / "robot" / "sdk"
SDK_PACKAGE = "wirepod_vector_sdk==0.8.1"
ASSET_DIR_IN_WHEEL = "anki_vector/opengl/assets"
PINNED = {
    "vector.obj": "db41c97d001f3ba1e4b61205518dbe52baf867d924ba482cdec496acc1992cda",
    "vector.mtl": "902e70e2326e503829cfced9742fac863c2be9f181a0401270fb8dd5ff68ab23",
    "LICENSE.txt": "cb77eb27637896ceea9dd9c48fb123368afbe964fbbacacd60f90983f1a60d32",
}
CUBE_FILES = {
    "cube.obj": "312c115cb952bcac3ad0e9c87067c4c1c6c8bfc2e712511937ff7d01f51016db",
    "cube.jpg": "9cd40878740f1f15776b4c86e1c3851a74be4ea60585b372944efa54233f693d",
}
# How the SDK's viewer poses the parts (anki_vector/opengl/opengl_vector.py, Apache-2.0).
# Lengths are in the model's units (centimetres) unless the name says mm. The viewer notes
# these are tuned to the model file and are not measurements of the robot.
VIEWER_POSE = {
    "unit_m": 0.01, "lift_arm_length_mm": 66.0, "lift_pivot_height_mm": 45.0,
    "lift_angle_in_default_pose_rad": -0.1136, "fork_pivot": [3.0, 3.4], "upper_arm_pivot": [-3.73, 4.47],
    "lower_arm_pivot": [-3.74, 3.27], "head_pivot": [-1.1, 4.75],
}
DOWNLOAD_TIMEOUT_S = 240


def installed_assets() -> Optional[Path]:
    """The assets folder of an importable anki_vector, found without importing it."""
    spec = importlib.util.find_spec("anki_vector")
    if spec is None or not spec.submodule_search_locations:
        return None
    folder = Path(list(spec.submodule_search_locations)[0]) / "opengl" / "assets"
    return folder if (folder / "vector.obj").is_file() else None


def read_folder(folder: Path) -> Optional[Dict[str, bytes]]:
    if not all((folder / name).is_file() for name in PINNED):
        return None
    names = list(PINNED)
    if all((folder / name).is_file() for name in CUBE_FILES):
        names.extend(CUBE_FILES)
    return {name: (folder / name).read_bytes() for name in names}


def download_assets() -> Optional[Dict[str, bytes]]:
    """Download the SDK wheel (no install, no dependencies) and read the three files out of it."""
    with tempfile.TemporaryDirectory(prefix="vector-sdk-") as tmp:
        cmd = [sys.executable, "-m", "pip", "download", "--no-deps", "--only-binary=:all:", "--quiet",
               "--dest", tmp, SDK_PACKAGE]
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=DOWNLOAD_TIMEOUT_S)
        wheels = sorted(Path(tmp).glob("*.whl"))
        if done.returncode != 0 or len(wheels) != 1:
            print(f"pip download failed (rc={done.returncode}): {done.stderr.strip()[-400:]}", file=sys.stderr)
            return None
        with zipfile.ZipFile(wheels[0]) as archive:
            names = set(archive.namelist())
            members = {name: f"{ASSET_DIR_IN_WHEEL}/{name}" for name in PINNED}
            if not all(member in names for member in members.values()):
                return None
            optional = {name: f"{ASSET_DIR_IN_WHEEL}/{name}" for name in CUBE_FILES}
            if all(member in names for member in optional.values()):
                members.update(optional)
            return {name: archive.read(member) for name, member in members.items()}


def check_hashes(files: Dict[str, bytes]) -> Sequence[str]:
    """Names of the files whose hash is not the pinned one."""
    expected = {**PINNED, **CUBE_FILES}
    return [name for name, data in files.items() if hashlib.sha256(data).hexdigest() != expected[name]]


def write_copy(files: Dict[str, bytes], source: str) -> None:
    DEST.mkdir(parents=True, exist_ok=True)
    for name, data in files.items():
        (DEST / name).write_bytes(data)
    manifest = {"source": source, "sha256": {n: hashlib.sha256(d).hexdigest() for n, d in files.items()},
                "license": "Anki, Inc. Image and 3D Model License Agreement (LICENSE.txt beside this file)",
                "viewer_pose": VIEWER_POSE}
    (DEST / "manifest.json").write_text(json.dumps(manifest, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {len(files) + 1} files to {DEST.relative_to(HERE)} (from {source})")


def remove_copy() -> int:
    removed = 0
    for name in [*PINNED, *CUBE_FILES, "manifest.json"]:
        path = DEST / name
        if path.is_file():
            path.unlink()
            removed += 1
    if DEST.is_dir() and not any(DEST.iterdir()):
        DEST.rmdir()
    print(f"removed {removed} files")
    return 0


def main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--assets", type=Path, help="folder holding vector.obj, vector.mtl and LICENSE.txt")
    group.add_argument("--download", action="store_true", help=f"download {SDK_PACKAGE} from PyPI (not installed)")
    group.add_argument("--remove", action="store_true", help="delete the copy in public/robot/sdk")
    parser.add_argument("--accept-other-version", action="store_true", help="copy even if the hashes differ")
    args = parser.parse_args(list(argv))
    if args.remove:
        return remove_copy()
    if args.download:
        files, source = download_assets(), f"{SDK_PACKAGE} wheel from PyPI"
        if files is None:
            return 5
    else:
        folder = args.assets if args.assets is not None else installed_assets()
        files = read_folder(folder) if folder is not None else None
        source = f"folder {folder.name}" if args.assets is not None else "installed anki_vector package"
    if files is None:
        print("SDK model not found. Install the Vector SDK, pass --assets DIR, or use --download.", file=sys.stderr)
        return 3
    changed = check_hashes(files)
    if changed and not args.accept_other_version:
        print(f"hash differs from wirepod_vector_sdk 0.8.1 for: {', '.join(changed)} "
              "(pass --accept-other-version to copy anyway)", file=sys.stderr)
        return 4
    write_copy(files, source + (" (hashes differ from 0.8.1)" if changed else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

"""Which MuJoCo the simulacrum is fitted and trained on, and a check that a run used it.

The reference is 4.0.1, the version the browser build (web/, @mujoco/mujoco)
runs. Measured 2026-11-22: Linux and macOS 4.0.1 reproduce the browser's
fixtures to 1e-7 mm where 3.11.0 is up to 20 mm away, and against the real
robot the two versions tie (README, "Which MuJoCo: 4.0.1").

`check()` reads the version the bindings report AND the libmujoco file actually
mapped into this process, because a package directory can hold files from two
versions at once (the GPU host's own environment does). VECTOR_MUJOCO_EXPECT names
a different version for a run that must use one (that environment's 3.11.0, kept for mjlab).

    python -m simulacrum.mujoco_pin                      check this interpreter, JSON out
    python -m simulacrum.mujoco_pin --borrowed env/borrowed-training.json
                                                         also check the borrowed training packages
Exit 0 when every check passes, 3 when one does not, 2 on bad arguments.
"""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import importlib.metadata
import json
import os
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

PINNED_VERSION = "4.0.1"
EXPECT_ENV = "VECTOR_MUJOCO_EXPECT"
EXIT_OK, EXIT_USAGE, EXIT_ENV = 0, 2, 3
MAX_IMAGES = 20_000  # upper bound on loaded images walked on macOS
MAX_MAP_LINES = 200_000  # upper bound on /proc/self/maps lines read on Linux
_VERSION_RE = re.compile(r"^\d+\.\d+\.\d+$")
_LIB_RE = re.compile(r"libmujoco(?:\.so)?\.(\d+\.\d+\.\d+)(?:\.dylib)?$")


def expected_version() -> str:
    """The version this run must use: VECTOR_MUJOCO_EXPECT if set, else the pin."""
    want = os.environ.get(EXPECT_ENV, "").strip() or PINNED_VERSION
    if not _VERSION_RE.match(want):
        raise ValueError(f"{EXPECT_ENV} must look like 4.0.1, got {want!r}")
    return want


def _linux_images() -> List[str]:
    paths = set()
    with open("/proc/self/maps", encoding="utf-8") as maps:
        for count, line in enumerate(maps):
            if count >= MAX_MAP_LINES:
                raise RuntimeError("/proc/self/maps longer than expected")
            parts = line.split()
            if len(parts) >= 6 and "libmujoco" in parts[-1]:
                paths.add(parts[-1])
    return sorted(paths)


def _macos_images() -> List[str]:
    libsystem = ctypes.CDLL("/usr/lib/libSystem.B.dylib")
    libsystem._dyld_image_count.restype = ctypes.c_uint32
    libsystem._dyld_get_image_name.restype = ctypes.c_char_p
    libsystem._dyld_get_image_name.argtypes = [ctypes.c_uint32]
    count = int(libsystem._dyld_image_count())
    if count > MAX_IMAGES:
        raise RuntimeError("more loaded images than expected")
    names = (libsystem._dyld_get_image_name(i) for i in range(count))
    return sorted({n.decode() for n in names if n is not None and b"libmujoco" in n})


def loaded_libmujoco() -> Optional[List[str]]:
    """Paths of every libmujoco mapped into this process; None where the platform cannot say."""
    if sys.platform.startswith("linux"):
        return _linux_images()
    if sys.platform == "darwin":
        return _macos_images()
    return None


def check(want: Optional[str] = None) -> Dict[str, Any]:
    """Import mujoco (after anything the caller already imported) and say whether this is the wanted one."""
    want = want or expected_version()
    assert _VERSION_RE.match(want), f"not a MuJoCo version: {want!r}"
    import mujoco  # noqa: PLC0415  (late on purpose: tensordict must load first on the GPU host)

    package_dir = Path(mujoco.__file__).resolve().parent
    libs = loaded_libmujoco()
    problems = []
    if mujoco.mj_versionString() != want:
        problems.append(f"bindings report {mujoco.mj_versionString()}, want {want}")
    if libs is not None:
        if not libs:
            problems.append("no libmujoco is mapped into this process")
        for lib in libs:
            found = _LIB_RE.search(lib)
            if found is None or found.group(1) != want:
                problems.append(f"loaded {lib}, want libmujoco {want}")
            elif Path(lib).resolve().parent != package_dir:
                problems.append(f"loaded {lib} from outside the imported package {package_dir}")
    assert libs is None or all(isinstance(lib, str) for lib in libs), "library paths are strings"
    return {"want": want, "bindings": mujoco.mj_versionString(), "package": str(package_dir),
            "libmujoco": libs, "python": sys.executable, "ok": not problems, "problems": problems}


def check_borrowed(spec_file: Path) -> Dict[str, Any]:
    """Each borrowed package at its recorded version, with its dist-info RECORD byte-identical."""
    spec = json.loads(spec_file.read_text(encoding="utf-8"))
    packages = spec["packages"]
    assert 0 < len(packages) <= 64, "borrowed package list out of range"
    problems = []
    for name, pin in sorted(packages.items()):
        try:
            dist = importlib.metadata.distribution(name)
        except importlib.metadata.PackageNotFoundError:
            problems.append(f"{name} is not importable here")
            continue
        record = [f for f in (dist.files or []) if f.name == "RECORD"]
        digest = hashlib.sha256(record[0].read_binary()).hexdigest() if record else None
        if dist.version != pin["version"]:
            problems.append(f"{name} {dist.version}, want {pin['version']}")
        elif digest != pin["record_sha256"]:
            problems.append(f"{name} {dist.version} files differ from the recorded install")
    return {"borrowed": sorted(packages), "ok": not problems, "problems": problems}


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="Check which MuJoCo this interpreter runs.")
    parser.add_argument("--borrowed", type=Path, help="also check the packages listed in this file")
    args = parser.parse_args(argv)
    if args.borrowed is not None and not args.borrowed.is_file():
        print(f"no such file: {args.borrowed}", file=sys.stderr)
        return EXIT_USAGE
    try:
        want = expected_version()
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return EXIT_USAGE
    result = {"mujoco": check(want)}
    if args.borrowed is not None:
        result["training"] = check_borrowed(args.borrowed)
    ok = all(part["ok"] for part in result.values())
    print("MUJOCO_PIN " + json.dumps(result, sort_keys=True))
    return EXIT_OK if ok else EXIT_ENV


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

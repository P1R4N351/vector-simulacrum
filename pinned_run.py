"""Run one of this skill's files and fail the run if it did not use the pinned MuJoCo.

    python pinned_run.py <file in this skill> [args...]

run_in_ct.sh runs every file through this. Nothing is imported before the
file runs (on the GPU host importing mujoco before tensordict segfaults). When the
file ends, however it ends, simulacrum/mujoco_pin.check() reads the libmujoco
actually mapped into the process; if it is not the pinned version (or
VECTOR_MUJOCO_EXPECT's), the run exits 3 whatever the file returned. A file
that never imported mujoco gets it imported at the end, so the check still
says which one this interpreter would have used. One line, MUJOCO_PIN {...},
is printed last.
"""

from __future__ import annotations

import atexit
import json
import os
import runpy
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXIT_USAGE, EXIT_ENV = 2, 3


def _check_at_exit() -> None:
    sys.stdout.flush()
    sys.path.insert(0, str(HERE))
    from simulacrum import mujoco_pin  # noqa: PLC0415  (only after the file has run)

    try:
        result = mujoco_pin.check()
    except Exception as exc:  # noqa: BLE001  (any failure to check is a failed check, reported as such)
        result = {"ok": False, "problems": [f"the check itself failed: {exc!r}"]}
    assert isinstance(result.get("ok"), bool), "the check must say ok or not"
    assert result["ok"] or result.get("problems"), "a failed check must say why"
    print("MUJOCO_PIN " + json.dumps(result, sort_keys=True), flush=True)
    if not result["ok"]:
        print("not the pinned MuJoCo: " + "; ".join(result["problems"]), file=sys.stderr, flush=True)
        os._exit(EXIT_ENV)


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__.strip().splitlines()[2], file=sys.stderr)
        sys.exit(EXIT_USAGE)
    target = (HERE / sys.argv[1]).resolve()
    if HERE not in target.parents or not target.is_file() or target.suffix != ".py":
        print(f"not a Python file of this skill: {sys.argv[1]}", file=sys.stderr)
        sys.exit(EXIT_USAGE)
    atexit.register(_check_at_exit)
    sys.argv = [str(target)] + sys.argv[2:]
    sys.path[0] = str(target.parent)
    runpy.run_path(str(target), run_name="__main__")


if __name__ == "__main__":
    main()

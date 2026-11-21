"""Turn a capture_peers.py stream into image files and a manifest. Authored-by: CPCS (cpcs-66ffc23), 2026-11-21.

Usage: python3 hardware/save_capture.py <new output folder> < stream.jsonl

It writes <out>/frames/<n>.jpg for each frame row, numbered from 00001 in stream order, and
<out>/capture.jsonl: every row of the stream, with each frame's base64 JPEG replaced by its file name.
Each frame's bytes must hash to the sha256 the robot side computed, or nothing more is written and it
exits 4. It refuses an output folder that already holds anything (exit 2), and a stream with no
"begin" row or with frames before it (exit 4). It prints one summary line of JSON.
"""

import base64
import hashlib
import json
import os
import sys

MAX_LINES = 20000
MAX_LINE_BYTES = 4 * 1024 * 1024
MAX_FRAMES = 3000


def fail(code, why):
    sys.stdout.write(json.dumps({"ok": False, "why": why}) + "\n")
    return code


def save_frame(out, row, n):
    """Write one frame; return the manifest row, or None if its bytes do not match its sha256."""
    data = base64.b64decode(row["jpeg"], validate=True)
    if hashlib.sha256(data).hexdigest() != row.get("sha256") or data[:2] != b"\xff\xd8":
        return None
    name = "frames/%05d.jpg" % n
    with open(os.path.join(out, name), "xb") as fh:
        fh.write(data)
    kept = {k: v for k, v in row.items() if k != "jpeg"}
    kept["file"] = name
    return kept


def save(out, lines):
    """Returns (exit code, summary)."""
    os.makedirs(os.path.join(out, "frames"), exist_ok=False)
    kept, frames, ids, begun, end = [], 0, set(), False, None
    for i, line in enumerate(lines):
        if i >= MAX_LINES or len(line) > MAX_LINE_BYTES:
            return 4, {"why": "stream too long or a line too long", "line": i}
        if not line.strip():
            continue
        row = json.loads(line)
        if row.get("k") == "c" and row.get("name") == "begin":
            begun = True
        if row.get("k") == "f":
            if not begun or frames >= MAX_FRAMES or row.get("id") in ids:
                return 4, {"why": "frame before begin, too many frames, or a repeated image id", "line": i}
            entry = save_frame(out, row, frames + 1)
            if entry is None:
                return 4, {"why": "frame bytes do not match their sha256", "line": i}
            frames, row = frames + 1, entry
            ids.add(entry["id"])
        if row.get("k") == "c" and row.get("name") == "end":
            end = row.get("why")
        kept.append(row)
    with open(os.path.join(out, "capture.jsonl"), "x") as fh:
        fh.writelines(json.dumps(r, separators=(",", ":")) + "\n" for r in kept)
    if not begun:
        return 4, {"why": "no begin row"}
    return 0, {"frames": frames, "rows": len(kept), "end": end}


def main(argv):
    if len(argv) != 2:
        return fail(2, "usage: save_capture.py <new output folder> < stream.jsonl")
    out = argv[1]
    if os.path.exists(out) and (not os.path.isdir(out) or os.listdir(out)):
        return fail(2, "output folder exists and is not empty")
    os.makedirs(out, exist_ok=True)
    code, summary = save(out, sys.stdin)
    summary.update(ok=code == 0, out=out)
    sys.stdout.write(json.dumps(summary) + "\n")
    return code


if __name__ == "__main__":
    sys.exit(main(sys.argv))

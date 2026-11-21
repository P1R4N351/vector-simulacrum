"""Pair overhead and robot camera frames by nearest host receipt time."""

import csv, json, sys
from pathlib import Path
import cv2
import numpy as np
from subprocess import Popen, PIPE

root = Path(sys.argv[1])
events = list(map(json.loads, (root / sys.argv[2]).open()))
start, end = map(float, sys.argv[3:5])
dest = Path(sys.argv[5])


def times(folder):
    return [(float(f.stem.split("-")[0]), f) for f in sorted(folder.glob("*.jpg"))]


top, on = times(root / "frames"), times(root / "on")
states = [r for r in events if r.get("name") == "robot_state"]
actions = [r for r in events if r["kind"] in ("begin", "pulse_begin", "pulse_end")]
cmd = [
    "ffmpeg",
    "-loglevel",
    "error",
    "-y",
    "-f",
    "rawvideo",
    "-pix_fmt",
    "bgr24",
    "-s",
    "960x420",
    "-r",
    "8",
    "-i",
    "-",
    "-c:v",
    "libx264",
    "-crf",
    "20",
    "-pix_fmt",
    "yuv420p",
    "-movflags",
    "+faststart",
    str(dest),
]
with Popen(cmd, stdin=PIPE) as movie, dest.with_suffix(".csv").open("w") as out:
    w = csv.writer(out, lineterminator="\n")
    w.writerow(
        [
            "sec",
            "overhead_host_s",
            "onboard_host_s",
            "capture_ms",
            "image_id",
            "state_host_s",
        ]
    )
    for t in np.arange(start, end, 0.125):
        ht, hf = min(top, key=lambda p: abs(p[0] - t))
        ot, of = min(on, key=lambda p: abs(p[0] - t))
        st = min(states, key=lambda r: abs(r["t"] - t))
        d = st["data"]
        left = cv2.resize(cv2.imread(str(hf))[:510, :680], (480, 360))
        right = cv2.resize(cv2.imread(str(of)), (480, 360))
        if abs(ot - t) > 0.7:
            right[:] = 0
        canvas = cv2.copyMakeBorder(
            np.hstack([left, right]), 28, 32, 0, 0, cv2.BORDER_CONSTANT
        )
        for x, label in [(8, "See3CAM"), (488, "Unit 3 camera")]:
            cv2.putText(
                canvas, label, (x, 21), 0, 0.60, (245, 245, 245), 1, cv2.LINE_AA
            )
        last = next((r for r in reversed(actions) if r["t"] <= t), {})
        text = f"{t - start:.2f} s   {last.get('action', '')}   Lift {d['lift_height_mm']:.1f} mm   Carrying ID {d.get('carrying_object_id', -1)}"
        cv2.putText(canvas, text, (8, 411), 0, 0.60, (245, 245, 245), 1, cv2.LINE_AA)
        movie.stdin.write(canvas.tobytes())
        w.writerow([t - start, ht, ot, *of.stem.split("-")[1:], st["t"]])
    movie.stdin.close()
    if movie.wait():
        raise RuntimeError("Video encoding failed")

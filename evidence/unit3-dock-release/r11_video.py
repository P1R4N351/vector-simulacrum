"""Export the first ten seconds of the R11 undock from original See3CAM recordings."""
import argparse
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "unit3-wheelstand/twin-timing"))
from shelf_view import SERIALS, ShelfView

parser = argparse.ArgumentParser()
parser.add_argument("archive", type=Path, help="R11 NUT pair and clocks.csv")
args = parser.parse_args()
clock = np.genfromtxt(args.archive / "clocks.csv", names=True, delimiter=",")
# Same telemetry origin as R11/state.csv.gz; host alignment bounded in R11/README.md.
origin = 1795316753.4585757 - 15.635229
shift = np.median(clock["unix_s"] - clock["monotonic_s"]) - origin
grid = np.arange(151) / 15
panels = []
for serial in SERIALS:
    path = args.archive / (serial + ".nut")
    pts = np.fromstring(subprocess.check_output(["ffprobe", "-v", "error", "-show_entries",
                          "packet=pts_time", "-of", "csv=p=0", str(path)], text=True), sep="\n") + shift
    assert pts[0] <= grid[0] and pts[-1] >= grid[-1] and np.all(np.diff(pts) > 0)
    cap = cv2.VideoCapture(str(path))
    ok, frame = cap.read()
    assert ok
    view = ShelfView(serial, frame)
    i = 0
    frames = []
    for t in grid:
        target = np.argmin(abs(pts - t))
        while i < target:
            ok, frame = cap.read()
            assert ok
            i += 1
        frames.append(cv2.resize(view.apply(frame), (600, 264), interpolation=cv2.INTER_AREA))
    panels.append(frames)
    cap.release()
cmd = ["ffmpeg","-loglevel","error","-y","-f","rawvideo","-pix_fmt","bgr24","-s","1200x264",
       "-r","15","-i","-","-an","-c:v","libx264","-crf","18","-pix_fmt","yuv420p","-movflags","+faststart",str(HERE/"r11-undock.mp4")]
with subprocess.Popen(cmd, stdin=subprocess.PIPE) as proc:
    for i, t in enumerate(grid):
        frame = np.hstack([panel[i] for panel in panels])
        cv2.putText(frame, f"R11 undock  {t:.2f} s", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, .65, (255,255,255), 2)
        proc.stdin.write(frame.tobytes())
    proc.stdin.close()
    assert proc.wait() == 0

"""Replay each fixed-posture spin from rest with the two existing 42 mm models."""

import csv
import json
from pathlib import Path

import numpy as np

from replay_endurance_drift import relative_center
from replay_measured_turn import run
from replay_pivot import configure_round

DATA = Path(__file__).resolve().parent / "evidence/lift-camera"


def main(data=DATA):
    result = []
    for turn in json.loads((data / "turns.json").read_text()):
        rows = turn["rows"]
        t = np.array([r["t"] for r in rows])
        wheels = np.array([[r["wl"], r["wr"]] for r in rows])
        sdk = relative_center(
            np.array([r["pose"][:2] for r in rows]), np.array([r["yaw"] for r in rows])
        )
        result.append([turn["label"], "SDK", *sdk[-1]])
        for name, step, shape in [
            ("Cylinder", 1, None),
            ("Sphere", 0.125, configure_round),
        ]:
            _, out = run(
                rows,
                t,
                wheels,
                42,
                dense=True,
                include_pose=True,
                step_ms=step,
                configure=shape,
            )
            delta = relative_center(out[:, 3:5], out[:, 5])[-1]
            result.append([turn["label"], name, *delta])
    with (data / "replays.csv").open("w") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["turn", "source", "forward_mm", "left_mm"])
        writer.writerows(result)


if __name__ == "__main__":
    main()

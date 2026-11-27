"""Extract the SDK wheel-speed fields paired with the existing mount trajectories."""

import argparse
import json
import gzip
import csv
import math
from pathlib import Path
from earlier_inputs import HERE, TRIALS


def extract(raw):
    with gzip.open(HERE / "approach-wheels.csv.gz", "wt") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow("trial t_s left_mmps right_mmps".split())
        for name in TRIALS:
            states = []
            for line in (raw / f"p2wheel-{name}.jsonl").read_text().splitlines():
                try:
                    r = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if r.get("kind") == "robot_state":
                    states.append(r)
            z = next(
                r["t"]
                for r in states
                if 45 < math.degrees(r["data"]["pose_pitch_rad"]) < 85
            )
            for r in states:
                t = r["t"] - z
                if -3.2 < t < 0.31:
                    w.writerow(
                        [
                            name,
                            t,
                            *[
                                r["data"].get(k + "_wheel_speed_mmps", 0)
                                for k in ("left", "right")
                            ],
                        ]
                    )


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("raw", type=Path)
    extract(ap.parse_args().raw)

"""Extract relative SDK paths, observed cube poses and radio counters around first mount."""

import argparse
import csv
import gzip
import json
import math
from pathlib import Path
from contextlib import ExitStack

TRIALS = ("W4", "W8", "W25", "W26", "W27", "W29", "W30")
HERE = Path(__file__).resolve().parent


def extract(raw):
    with ExitStack() as stack:

        def table(file, fields):
            w = csv.writer(
                stack.enter_context(gzip.open(HERE / file, "wt")), lineterminator="\n"
            )
            w.writerow(fields.split())
            return w

        paths = table(
            "earlier-path.csv.gz", "trial t_s x_mm y_mm yaw_rad pitch_rad lift_mm"
        )
        poses = table("earlier-pose.csv.gz", "trial t_s x_mm y_mm yaw_rad")
        packets = table("earlier-packets.csv.gz", "trial t_s tap")
        for trial in TRIALS:
            rows = []
            for line in (raw / f"p2wheel-{trial}.jsonl").read_text().splitlines():
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
            states = [r for r in rows if r.get("kind") == "robot_state"]
            crossing = next(
                r for r in states if 45 < math.degrees(r["data"]["pose_pitch_rad"]) < 85
            )
            z = crossing["t"]
            origin = crossing["data"]["pose"]["origin_id"]
            obs = [
                r
                for r in rows
                if r.get("kind") == "robot_observed_object"
                and z - 12 < r["t"] < z
                and r["data"].get("object_type") == "BLOCK_LIGHTCUBE1"
                and r["data"]["pose"].get("origin_id") == origin
            ][-1]
            p = obs["data"]["pose"]
            w, x, y, s = [p.get(f"q{i}", 0) for i in range(4)]
            poses.writerow(
                [
                    trial,
                    obs["t"] - z,
                    p["x"],
                    p["y"],
                    math.atan2(2 * (w * s + x * y), 1 - 2 * (s * s + y * y)),
                ]
            )
            window = [r for r in states if -3.2 < r["t"] - z < 0.31]
            assert window[0]["t"] - z <= -3 and window[-1]["t"] - z >= 0.105
            for r in window:
                d = r["data"]
                p = d["pose"]
                assert p["origin_id"] == origin
                paths.writerow(
                    [
                        trial,
                        r["t"] - z,
                        p.get("x", 0),
                        p.get("y", 0),
                        d["pose_angle_rad"],
                        d["pose_pitch_rad"],
                        d["lift_height_mm"],
                    ]
                )
            for line in (raw / f"p2wheel-{trial}.tsv").read_text().splitlines():
                ns, _, payload = line.split()
                b = bytes.fromhex(payload)
                t = int(ns) * 1e-9 - z
                if -1.2 < t < 0.35 and len(b) == 20 and b[0] == 2:
                    packets.writerow([trial, t, b[1]])


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("raw", type=Path)
    extract(ap.parse_args().raw)

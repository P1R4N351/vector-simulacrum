"""Extract radio accelerometer slots around the same SDK pitch crossing as earlier_inputs."""

import argparse
import csv
import gzip
import json
import math
from pathlib import Path
import struct
from earlier_inputs import HERE, TRIALS


def extract(raw):
    with (
        gzip.open(HERE / "impact-samples.csv.gz", "wt") as out,
        (HERE / "impact-initial-poses.csv").open("w") as poses,
    ):
        pose_writer = csv.writer(poses, lineterminator="\n")
        pose_writer.writerow("trial t_s x_mm y_mm yaw_rad".split())
        writer = csv.writer(out, lineterminator="\n")
        writer.writerow("trial packet_s slot x y z".split())
        for name in TRIALS:
            observations = []
            for line in (raw / f"p2wheel-{name}.jsonl").read_text().splitlines():
                try:
                    row = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if (
                    row.get("kind") == "robot_observed_object"
                    and row["data"].get("object_type") == "BLOCK_LIGHTCUBE1"
                ):
                    observations.append(row)
                if (
                    row.get("kind") == "robot_state"
                    and 45 < math.degrees(row["data"]["pose_pitch_rad"]) < 85
                ):
                    zero = row["t"]
                    break
            else:
                raise ValueError(name)
            origin = row["data"]["pose"]["origin_id"]
            obs = max(
                (
                    r
                    for r in observations
                    if -12 < r["t"] - zero <= -3
                    and r["data"]["pose"].get("origin_id") == origin
                ),
                key=lambda r: r["t"],
            )
            pose = obs["data"]["pose"]
            w, x, y, z = [pose.get(f"q{i}", 0) for i in range(4)]
            pose_writer.writerow(
                [
                    name,
                    obs["t"] - zero,
                    pose["x"],
                    pose["y"],
                    math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
                ]
            )
            for line in (raw / f"p2wheel-{name}.tsv").read_text().splitlines():
                ns, _, payload = line.split()
                t = int(ns) * 1e-9 - zero
                b = bytes.fromhex(payload)
                if -7 <= t <= 0.35 and len(b) == 20 and b[0] == 2:
                    for slot in range(3):
                        writer.writerow(
                            [name, t, slot, *struct.unpack_from("<3h", b, 2 + 6 * slot)]
                        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw", type=Path)
    extract(parser.parse_args().raw)

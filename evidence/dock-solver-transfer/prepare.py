"""Extract twelve signed speed profiles in their recorded clocks."""
import csv
import json
import math
from itertools import groupby
from pathlib import Path


def rows(path):
    with Path(path).open() as source:
        return list(csv.DictReader(source))


with open("inputs.csv", "w") as output:
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow("unit,turn,time_s,left,right,yaw_dps,head_deg,lift_mm,phase".split(","))
    native = rows("../hw-rec-20261119/unit2-native-replay-input.csv")
    for name, group in groupby(native, lambda row: row["turn"]):
        samples = list(group)
        for r in samples:
            t = float(r["ms_from_last_edge"]) / 1000
            phase = "steady" if -0.7 <= t < -0.25 else "stop" if -0.10 <= t <= 0.25 else "other"
            writer.writerow(["unit2", name, t, r["left_mmps"], r["right_mmps"],
                             math.degrees(float(r["head_gz"])), math.degrees(float(r["head_rad"])), 30.645, phase])
    base = "../workshop-spins/speed-holdout/"
    states = rows(base + "states.csv")
    events = rows(base + "events.csv")
    for command, stop in zip([e for e in events if e["event"] == "spin"],
                             [e for e in events if e["event"] == "stop"]):
        start, end = float(command["time_s"]), float(stop["time_s"])
        for r in states:
            t = float(r["time_s"])
            if start - 0.12 <= t <= end + 0.55:
                phase = "steady" if start+0.45 <= t < end-0.10 else "stop" if end <= t <= end+0.35 else "other"
                writer.writerow(["unit3", command["detail"], t-end, r["left_mm_s"], r["right_mm_s"],
                                 math.degrees(float(r["gyro_z"])), math.degrees(float(r["head_rad"])),
                                 r["lift_mm"], phase])
    with open("../hw-rec-20261118/run-e-spins.jsonl") as source:
        records = [json.loads(line) for line in source]
    for index, command in enumerate(records):
        if command.get("name") != "wheels":
            continue
        stop = next(r for r in records[index+1:] if r.get("name") == "stop")
        start, end = command["t"], stop["t"]
        for r in records:
            if r["k"] == "s" and start-0.12 <= r["t"] <= end+0.55:
                t = r["t"]
                phase = "steady" if start+0.45 <= t < end-0.10 else "stop" if end <= t <= end+0.35 else "other"
                writer.writerow(["unit1",command["label"],t-end,r["wl"],r["wr"],math.degrees(r["gyro"][2]),
                                 math.degrees(r["head"]),r["lift"],phase])

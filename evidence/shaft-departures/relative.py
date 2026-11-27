"""Predict a relative seat correction from pre-motion silhouettes alone."""
import csv
import io
import json
import statistics
import subprocess
import sys

def rows(path):
    with open(path) as stream:
        return list(csv.DictReader(stream))

depths = {r["session"]: float(r["start_depth_mm"]) for r in rows(sys.argv[1])}
reference = statistics.median(depths[name] for name in "ABC")
limits = {r["session"]: float(r["last_on_s"]) for r in rows(sys.argv[2])}
with open(sys.argv[3]) as stream:
    trials = json.load(stream)
with open("relative.csv", "w") as stream:
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(["session","seat","shift_mm","depth_minus_reference_mm","off_ms","signed_miss_ms"])
    for trial in trials:
        name = trial["name"]
        if name not in depths:
            continue
        delta = depths[name] - reference
        for seat, shift in [("fixed", 6), ("relative", 6-delta)]:
            output = subprocess.check_output(["node", "replay.mjs", str(shift)],
                                              input=json.dumps([trial]), text=True)
            off = next(float(r["t"]) for r in csv.DictReader(io.StringIO(output)) if r["on"] == "0")
            miss = off - min(0, max(limits[name], off))
            writer.writerow([name,seat,shift,delta,1000*off,1000*miss])

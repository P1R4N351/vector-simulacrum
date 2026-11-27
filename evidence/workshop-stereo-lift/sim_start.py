"""Replay the held-out physical command sequence and export pre-command simulated heights."""
import csv
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory
from statistics import median

here=Path(__file__).resolve().parent
runner=here.parent/"workshop-joints-20261127/lift-duration/replay.mjs"
with TemporaryDirectory() as folder:
    path=Path(folder)
    with (here/"recovery/sdk.csv").open() as f:
        rows=list(csv.DictReader(f))
    with (path/"recording.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["s","event","target_mm","duration_s","lift_mm","head_deg"])
        for r in rows:
            w.writerow([r["relative_s"],r["event"],r["target"],.6,r["lift_mm"],r["head_deg"]])
    subprocess.run(["node",str(runner),path.as_uri()+"/"],check=True)
    with (path/"replay.csv").open() as f:
        samples=list(csv.DictReader(f))
    with (here/"prelower-sim.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n");w.writerow(["command_s","sim_initial_mm"])
        for r in rows:
            if r["event"]=="command" and r["target"]=="52":
                t=float(r["relative_s"])
                w.writerow([t,median(float(s["lift_mm"]) for s in samples if t-.2<=float(s["s"])<t)])

import csv
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

here=Path(__file__).resolve().parent
runner=here.parent/"workshop-joints-20261127/lift-duration/replay.mjs"
with TemporaryDirectory() as directory:
    work=Path(directory)
    with (here/"sdk.csv").open() as src, (work/"recording.csv").open("w") as dest:
        w=csv.writer(dest,lineterminator="\n")
        w.writerow(["s","event","target_mm","duration_s","lift_mm","head_deg"])
        for r in csv.DictReader(src):
            w.writerow([r["relative_s"],r["event"],r["target"],0.65,r["lift_mm"],r["head_deg"]])
    subprocess.run(["node",str(runner),work.as_uri()+"/"],check=True)
    for name in ("returns.csv","replay.csv"):
        (here/name).write_bytes((work/name).read_bytes())

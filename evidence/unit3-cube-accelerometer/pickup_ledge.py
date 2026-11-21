"""Replay independently photographed cube profiles on the recorded acquisition."""
import csv
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

import free_pickup

repo = free_pickup.repo
sys.path.insert(0, str(repo / "evidence/unit3-cube-ledge"))
from compare import make

source = Path(sys.argv[2]).resolve()
output = free_pickup.here
profiles = ("original", "ledge", "feet", "panel", "all", "shallow", "deep")
with TemporaryDirectory() as temp, (output / "pickup-ledge.csv").open("w") as out:
    temp = Path(temp)
    for filename in ("pickup-path.csv", "pickup-sightings.csv", "pickup.tsv", "action-times.csv"):
        (temp / filename).symlink_to(output / filename)
    writer = csv.writer(out, lineterminator="\n")
    writer.writerow(["profile", "initialization", "slide_2p86_mm", "max_center_mm", "held_floor_fraction"])
    free_pickup.here = temp
    for profile in profiles:
        path = make(source, profile, temp)
        sys.argv[1] = str(path)
        free_pickup.main()
        samples = np.genfromtxt(temp / 'free-pickup.csv', delimiter=',', names=True, encoding=None, dtype=None)
        for name in dict.fromkeys(samples["initialization"]):
            run = samples[samples["initialization"] == name]
            near = run[np.argmin(abs(run["time_s"]-2.86))]
            hold = run[(run["time_s"]>6.5) & (run["time_s"]<8.3)]
            writer.writerow([profile, name, round(np.hypot(near["cube_dx_mm"], near["cube_dy_mm"]),3),
                             round(max(run["center_z_mm"]),3), hold["touches_floor"].mean()])

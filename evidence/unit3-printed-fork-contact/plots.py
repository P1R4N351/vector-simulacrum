from pathlib import Path
import csv
import gzip
import matplotlib.pyplot as plt
import numpy as np
from tilt import poses, p

here = Path(__file__).resolve().parent
with (here / "scores.csv").open() as f:
    scores = list(csv.DictReader(f))
with gzip.open(here / "trajectories.csv.gz", "rt") as f:
    traces = list(csv.DictReader(f))
fig, (a, b) = plt.subplots(1, 2, figsize=(11.4, 4))


def selected(row, geometry):
    return (
        row["fork"] == geometry
        and row["heading"] == "overhead"
        and row["drive"] == "weld"
        and row["height_offset"] == "0"
    )


for geom, label, color in [
    ("boxes", "Three boxes", "#9199a0"),
    ("stl", "Printed fork STL", "#006a76"),
]:
    rows = [r for r in scores if selected(r, geom)]
    a.plot(
        [int(r["trial"]) for r in rows],
        [float(r["vector_error_mm"]) for r in rows],
        "o-",
        color=color,
        label=label,
    )
    rows = [r for r in traces if selected(r, geom) and r["trial"] == "1"]
    b.plot(
        [float(r["time"]) - 1.5 for r in rows],
        [float(r["tilt_deg"]) for r in rows],
        color=color,
        label=label,
    )
a.set(
    yscale="log",
    xticks=range(1, 6),
    xlabel="Shelf trial",
    ylabel="Endpoint vector error (mm)",
    title="Same starting heading and wrist drive",
)
a.set_yticks([1, 2, 5, 10, 25], labels=["1", "2", "5", "10", "25"])
a.grid(alpha=0.2)
a.legend(frameon=False)
rows = p[p["trial"] == 1]
for k, f, label, marker in [
    (-1.499, 473.11, "See3CAM lens fit", "o"),
    (0, 700, "Pinhole sensitivity", "+"),
]:
    r = poses(rows, k, f)
    b.scatter(r[:, 1], r[:, 2], s=22, marker=marker, label=label)
b.set(
    xlim=(3.5, 7.1),
    ylim=(-1, 45),
    xlabel="Seconds from approach command",
    ylabel="Tilt from starting top-face normal (°)",
    title="Trial 1 during lift and release",
)
b.legend(fontsize=8, frameon=False)
b.grid(alpha=0.2)
fig.tight_layout()
fig.savefig(here / "fork-comparison.png", dpi=160)

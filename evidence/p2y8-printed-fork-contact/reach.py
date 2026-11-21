"""Use pre-lift motion on trials 2–3 to choose a common wrist reach offset."""
import csv
from concurrent.futures import ProcessPoolExecutor
import matplotlib.pyplot as plt
import numpy as np
from evaluate import run, here
from push_phase import read

camera = read(here.parent / "p2y8-paired-cube-push/traces.csv")
windows = read(here / "push-phase.csv")


def score(job, *, cube_depth_mm=0):
    trial, reach = job
    result, samples = run((trial, "stl", "overhead", "weld", cube_depth_mm, 0, 0), extra_reach_mm=reach)
    trace = np.asarray(samples)
    start = windows["raise_sec"][trial - 1]
    obs = camera[(camera["trial"] == trial) & (camera["camera"] == "frames") &
                 (camera["seconds"] >= start - 1) & (camera["seconds"] <= start - .2)]
    measured = [np.median(obs[col]) for col in ("forward_mm", "left_mm")]
    predicted = [np.median(np.interp(obs["seconds"], trace[:, 0] - 1.5, trace[:, col]))
                 for col in (1, 2)]
    return trial, reach, np.linalg.norm(np.subtract(predicted, measured)), result[-3], result[-1]


if __name__ == "__main__":
    offsets = (0, .5, 1, 1.5, 2, 2.5, 3)
    with ProcessPoolExecutor(4) as pool:
        fitted = list(pool.map(score, [(trial, r) for trial in (2, 3) for r in offsets]))
        best = min(offsets, key=lambda r: np.mean([s[2] for s in fitted if s[1] == r]))
        checks = list(pool.map(score, [(trial, r) for trial in (4, 5, 1) for r in sorted({0, best})]))
    rows = fitted + checks
    with (here / "reach.csv").open("w") as f:
        out = csv.writer(f, lineterminator="\n")
        out.writerow(("trial", "extra_reach_mm", "push_error_mm", "end_error_mm", "peak_tilt_deg"))
        out.writerows(rows)
    fig, axs = plt.subplots(1, 2, figsize=(10, 4))
    for col, (ax, title) in enumerate(zip(axs, ("Before lift", "Settled endpoint")), 2):
        for r, style in ((0, "o"), (best, "x")):
            series = sorted(s for s in rows if s[1] == r)
            ax.scatter([s[0] for s in series], [s[col] for s in series], marker=style, s=55,
                       label=f"+{r:g} mm")
        ax.set(title=title, xlabel="Shelf trial", xticks=range(1, 6), ylabel="2D error (mm)")
        ax.legend(title="Fork reach")
    fig.tight_layout()
    fig.savefig(here / "reach.png", dpi=150)
    print("Selected offset:", best, "Checks:", checks)

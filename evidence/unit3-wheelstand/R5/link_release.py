"""Compare 4- and 8-substep replays in baseline, muted order for each step."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

from pickup_onset import action_time, read

here = Path(__file__).resolve().parent
start = action_time(here / "actions.csv", "pickup for inward transport", "sec")
lights = read(here / "raise-lights.csv")
glyph = read(here / "glyph-raise.csv")
frames = [91, 94]
times = [float(lights["receipt_s"][lights["frame"] == f].item()) - start for f in frames]
fig, axes = plt.subplots(1, 2, sharey=True, figsize=(9, 4), layout="constrained")
rows = []
pieces = []
for step, ax, baseline, muted in zip((4, 8), axes, sys.argv[1::2], sys.argv[2::2], strict=True):
    for label, folder in (("Full contact", baseline), ("Links disabled at raise", muted)):
        directory = Path(folder)
        pose = read(directory / "pickup-replay.csv")
        before = pose[pose["sec"] < 2.559]
        if folder == baseline:
            reference = before
        else:
            np.testing.assert_array_equal(reference, before)
            assert len((directory / "link-patch.csv").read_text().splitlines()) == 1
        window = (pose["sec"] > 2.5) & (pose["sec"] < 2.86)
        ax.plot(pose["sec"][window], pose["tilt_deg"][window], label=label)
        rows.append([1/step, label, *np.interp(times, pose["sec"], pose["tilt_deg"]),
                     np.interp(4, pose["sec"], pose["center_height_mm"])])
    patch = read(Path(baseline) / "link-patch.csv")
    for rail, cube in sorted(set(zip(patch["rail"], patch["cube"]))):
        hits = patch[(patch["rail"] == rail) & (patch["cube"] == cube)]
        w = hits["normal_Ns"]
        centroid = [np.average(hits[k], weights=w) for k in ("x_mm", "y_mm", "z_mm")]
        pieces.append([1/step, rail, cube, w.sum(), w.sum()/patch["normal_Ns"].sum(), *centroid])
    g = glyph[np.isin(glyph["frame"], frames)]
    y = g["normal_change_deg"]
    ax.errorbar(times, y, yerr=[y-g["jitter_p05_deg"], g["jitter_p95_deg"]-y],
                fmt="ko", capsize=4, label="Filmed glyph change ± jitter range")
    ax.axvline(2.559, ls=":", color=".5")
    ax.set(title=f"{1/step:g} ms solver step", xlabel="Seconds after pickup command")
axes[0].set_ylabel("Simulated tilt / filmed normal change (°)")
axes[0].legend(fontsize=7)
fig.savefig(here / "link-release.png", dpi=160)
for name, header, data in [
    ("link-release", "step_ms mode tilt_frame91_deg tilt_frame94_deg center_at_4s_mm", rows),
    ("link-location", "step_ms rail cube normal_Ns impulse_fraction x_mm y_mm z_mm", pieces),
]:
    with (here / f"{name}.csv").open("w") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(header.split())
        writer.writerows(data)

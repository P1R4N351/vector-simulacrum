"""Compare isolated fork-pad sliding friction at two time steps with the filmed raise."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

from pickup_onset import read

here = Path(__file__).resolve().parent
optical = read(here / "height-transfer.csv")
timing = read(here / "approach-links.csv")
timing = timing[(timing["step_ms"] == .125) & (timing["link_condition"] == "Full contact")]
times = timing["receipt_s_since_cmd"]
rows = []
fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True, sharey=True, layout="constrained")
for i, ax in enumerate(axes):
    dirs = list(map(Path, sys.argv[1+4*i:5+4*i]))
    base = read(dirs[0] / "pickup-replay.csv")
    for label, mu, path in zip(("Original", "Split control", "Half", "Double"),
                               (.9, .9, .45, 1.8), dirs, strict=True):
        material = read(path / "fork-materials.csv")
        for m in material:
            expected = mu if m["material"] == "rubber" else .5
            np.testing.assert_allclose([m["mu_tangent1"],m["mu_tangent2"]], expected)
        assert set(material["material"]) == {"rubber", "shell"}
        pose = read(path / "pickup-replay.csv")
        height = pose["center_height_mm"]
        rise = height - np.interp(times[0], pose["sec"], height)
        window = (pose["sec"] > times[0]-.05) & (pose["sec"] < times[-1]+.05)
        ax.plot(pose["sec"][window],rise[window],label=f"{label} · μ {mu:g}")
        shift = np.max(abs(height-base["center_height_mm"]))
        for frame, t in zip(timing["frame"],times,strict=True):
            rows.append([(0.25,0.125)[i],label,mu,int(frame),np.interp(t,pose["sec"],rise),shift])
    for width in (20,23):
        g = optical[optical["glyph_width_mm"] == width]
        ax.errorbar(times,g["rise_mm"],yerr=[g["rise_mm"]-g["rise_p05"],g["rise_p95"]-g["rise_mm"]],
                    fmt=".",capsize=2,label=f"Filmed · {width} mm glyph")
    ax.set(ylabel="Rise from frame 85 (mm)",title=f"{(0.25,0.125)[i]} ms solver step")
axes[0].legend(ncol=3,fontsize=8)
axes[-1].set_xlabel("Seconds from pickup command")
fig.savefig(here/"fork-pad-friction.png",dpi=160)
with (here/"fork-pad-friction.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("step_ms condition pad_mu frame center_rise_mm max_height_difference_from_original_mm".split())
    w.writerows(rows)

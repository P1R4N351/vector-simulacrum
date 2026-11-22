"""Check tool velocity at contact and compare free-cube rise with optical frames."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

from pickup_onset import read

here = Path(__file__).resolve().parent
frames = read(here/"approach-links.csv")
times = frames[(frames["step_ms"] == .125) & (frames["link_condition"] == "Full contact")]
optical = read(here/"height-transfer.csv")
fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 7), layout="constrained")
rows = []
for j, folder in enumerate(map(Path, sys.argv[1:])):
    step = (0.25, 0.125)[j//3]
    mode = ("Pose only", "Zero velocity control", "Measured velocity")[j%3]
    a = np.loadtxt(folder/"tool-velocity.csv", delimiter=",", skiprows=1)
    raise_mask = (a[:,0] >= 2.559) & (a[:,0] <= 3.266)
    speed, solver = [1000*np.linalg.norm(a[:,sl],axis=1) for sl in (slice(1,4),slice(4,7))]
    expected = a[:,1:4] if j%3 == 2 else np.zeros_like(a[:,1:4])
    np.testing.assert_allclose(a[:,4:], expected, atol=1e-8)
    window = (a[:,0] > 2.25) & (a[:,0] < 3.6)
    if j == 0:
        axes[0].plot(a[window,0],speed[window],color="black",label="Measured wrist path")
    if j in (0,2):
        axes[0].plot(a[window,0],solver[window],"--",label=mode)
    pose = read(folder/"pickup-replay.csv")
    rise = pose["center_height_mm"] - np.interp(times["receipt_s_since_cmd"][0],pose["sec"],pose["center_height_mm"])
    ax = axes[1+j//3]
    ax.plot(pose["sec"][window],rise[window],label=mode)
    impulse = read(folder/"contact-impulse.csv")
    approach_ns = float(impulse["normal_Ns"][impulse["phase"] == "approach"].item())
    raise_ns = float(impulse["normal_Ns"][impulse["phase"] == "raise"].item())
    for f in times:
        rows.append([step,mode,int(f["frame"]),np.interp(f["receipt_s_since_cmd"],pose["sec"],rise),
                     speed[raise_mask].max(),solver[raise_mask].max(),approach_ns,raise_ns])
for ax,step in zip(axes[1:],(.25,.125)):
    for width in (20,23):
        x = optical[optical["glyph_width_mm"]==width]
        ax.errorbar(times["receipt_s_since_cmd"],x["rise_mm"],yerr=[x["rise_mm"]-x["rise_p05"],x["rise_p95"]-x["rise_mm"]],
                    fmt=".",capsize=2,label=f"Filmed · {width} mm glyph")
    ax.set_ylabel(f"Center rise (mm)\n{step} ms step")
axes[0].set_ylabel("Wrist speed (mm/s)")
axes[-1].set_xlabel("Seconds from pickup command")
for ax in axes:
    ax.legend(fontsize=7,ncol=3)
fig.savefig(here/"velocity-transfer.png",dpi=160)
with (here/"velocity-transfer.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("step_ms mode frame rise_mm peak_path_speed_mm_s peak_solver_speed_mm_s approach_fork_normal_Ns raise_fork_normal_Ns".split())
    w.writerows(rows)

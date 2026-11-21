"""Score the recorded push before lift motion can change the marker projection."""
from pathlib import Path
import gzip
import csv
import numpy as np
import matplotlib.pyplot as plt

here = Path(__file__).resolve().parent


def read(path, **options):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf8", **options)


def main():
    camera = read(here.parent / "unit3-paired-cube-push/traces.csv")
    support = read(here / "support.csv")
    with gzip.open(here / "trajectories.csv.gz", "rt") as stream:
        model = read(stream)
    model = model[(model["fork"] == "stl") & (model["drive"] == "weld") &
                  (model["heading"] == "overhead") & (model["depth_offset"] == 0) &
                  (model["lateral_offset"] == 0) & (model["height_offset"] == 0)]
    fig, axes = plt.subplots(5, 2, figsize=(10, 10), sharex=True)
    rows = []
    for trial, panels in enumerate(axes, 1):
        obs = camera[(camera["trial"] == trial) & (camera["camera"] == "frames")]
        sim = model[model["trial"] == trial]
        arm = support[support["trial"] == trial]
        start = arm["sec"][arm["lift_mm"] > np.median(arm["lift_mm"][:100]) + 1][0]
        # Leave a timing margin before the encoder-defined raise.
        pre = (obs["seconds"] >= start - 1.0) & (obs["seconds"] <= start - 0.2)
        t = sim["time"] - 1.5
        fields = [trial, round(start, 3), int(sum(pre))]
        for ax, col in zip(panels, ("forward_mm", "left_mm")):
            pred = np.interp(obs["seconds"][pre], t, sim[col])
            fields.extend([np.median(obs[col][pre]), np.median(pred)])
            ax.plot(t, sim[col], color="#d05220", label="Printed-fork replay")
            ax.scatter(obs["seconds"], obs[col], s=10, color="#284f85", label="See3CAM glyph")
            ax.axvspan(start - 1, start - .2, color="#6ab68b", alpha=.24)
            ax.axvline(start, color="#555", lw=.7)
            ax.set_xlim(-1.6, 11.5)
            ax.grid(alpha=.2)
        fields.append(np.hypot(fields[3] - fields[4], fields[5] - fields[6]))
        rows.append(fields)
        panels[0].set_ylabel(f"Trial {trial} (mm)")
    axes[0, 0].legend(fontsize=8)
    axes[0, 0].set_title("Dock-forward displacement")
    axes[0, 1].set_title("Dock-left displacement")
    for ax in axes[-1]:
        ax.set_xlabel("Seconds from approach")
    fig.suptitle("Cube motion before the lift rises", fontsize=16)
    fig.tight_layout()
    fig.savefig(here / "push-phase.png", dpi=150)
    with (here / "push-phase.csv").open("w") as out:
        writer = csv.writer(out, lineterminator="\n")
        writer.writerow(["trial", "raise_sec", "frames", "camera_forward_mm", "sim_forward_mm",
                         "camera_left_mm", "sim_left_mm", "vector_error_mm"])
        writer.writerows(rows)


if __name__ == "__main__":
    main()

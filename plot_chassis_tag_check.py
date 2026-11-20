"""Compare independent optical chassis motion and equivalent static IMU pitch change."""

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent / "evidence"


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


ch = read(root / "face-tag-trial/chassis.csv")
acc = read(root / "head-sweep/samples.csv")
fig, axes = plt.subplots(2, 2, figsize=(10, 7), sharex=True)
vals = []
for block in sorted({r["block"] for r in ch}):
    group = [r for r in ch if r["block"] == block]
    sdk = []
    opt = []
    for r in group:
        samples = [s for s in acc if s["block"] == block and s["visit"] == r["visit"]]
        h = np.radians([float(s["head_deg"]) for s in samples])
        ax, az = np.array(
            [[float(s[k]) for s in samples] for k in ("ax_mm_s2", "az_mm_s2")]
        )
        sdk.append(
            np.degrees(np.arcsin(np.mean(ax * np.cos(h) - az * np.sin(h)) / 9810))
        )
        # Positive rotation about body-left pitches the forward axis DOWN.
        opt.append(-float(r["planar_fit_pitch_deg"]))
    sdk = np.array(sdk) - sdk[0]
    opt = np.array(opt) - opt[0]
    j = int(block[0] == "A")
    x = range(1, 7)
    axes[0, j].plot(x, sdk, "o-", ms=3, alpha=0.8, label=block)
    axes[1, j].plot(x, opt, "o-", ms=3, alpha=0.8)
    vals.append((block, sdk[3] - sdk[2], opt[3] - opt[2]))
for j in range(2):
    axes[0, j].set_title(["B · original sweep", "A · held-out spot"][j])
    axes[0, j].legend(fontsize=7, ncol=2)
    axes[1, j].set_xticks(range(1, 7), ["0", "25", "−15", "40", "10", "0"])
    axes[1, j].set_xlabel("Commanded head angle (°), in time order")
    for ax in axes[:, j]:
        ax.axhline(0, color=".6", lw=0.7)
        ax.set_ylim(-1.35, 1.9)
        ax.grid(alpha=0.2)
axes[0, 0].set_ylabel("IMU equivalent pitch change (°)")
axes[1, 0].set_ylabel("Apparent chassis pitch change (°)")
fig.suptitle("Archived untagged sweeps · relative to the first 0° rest")
fig.tight_layout()
fig.savefig(root / "physics-plots/chassis-pitch-optical.png", dpi=160)
for s in "BA":
    g = np.array([v[1:] for v in vals if v[0][0] == s])
    print(
        s,
        "low-to-high median, range (IMU and camera)",
        np.round(np.array([np.median(g, axis=0), g.min(0), g.max(0)]), 3),
    )

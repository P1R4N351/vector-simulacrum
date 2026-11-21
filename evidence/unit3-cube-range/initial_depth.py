"""Test the 2 mm contact hypothesis using independent pre-motion range samples."""
import csv
from concurrent.futures import ProcessPoolExecutor

import matplotlib.pyplot as plt
import numpy as np
from score import HERE, replay_trial


def compare(trial):
    baseline = replay_trial(trial)
    closer = replay_trial(trial, cube_depth_mm=-2)
    assert np.array_equal(baseline[:, :5], closer[:, :5])
    keep = ((baseline[:, 1] < 1.5) & (baseline[:, 4] == 1) &
            (baseline[:, 6] == 1) & (closer[:, 6] == 1))
    original = baseline[keep, 5]
    shifted = closer[keep, 5]
    measured = baseline[keep, 3]
    return (trial, len(measured), np.median(measured), np.median(original),
            np.median(shifted), np.mean(abs(original - measured)),
            np.mean(abs(shifted - measured)), np.percentile(measured, 25),
            np.percentile(measured, 75))


if __name__ == "__main__":
    with ProcessPoolExecutor(4) as pool:
        results = list(pool.map(compare, range(1, 6)))
    with (HERE / "initial-depth.csv").open("w") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["trial", "paired_rows", "observed_median_mm", "original_median_mm",
                         "closer_median_mm", "original_mae_mm", "closer_mae_mm",
                         "observed_q25_mm", "observed_q75_mm"])
        writer.writerows(results)
    trial, _, measured, original, shifted, _, _, q25, q75 = np.asarray(results).T
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.errorbar(trial - .13, measured, yerr=[measured-q25, q75-measured],
                fmt="o", capsize=4, color="#185e77", label="unit 3 median and middle 50%")
    ax.scatter(trial, original, marker="s", label="Camera placement")
    ax.scatter(trial + .13, shifted, marker="x", label="Cube 2 mm closer")
    ax.set(xticks=trial, xlabel="Shelf trial", ylabel="Reported range (mm)",
           title="Cube distance before the first drive command")
    ax.legend()
    fig.tight_layout()
    fig.savefig(HERE / "initial-depth.png", dpi=160)

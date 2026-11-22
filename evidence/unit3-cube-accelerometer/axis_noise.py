"""Test axis-specific stationary noise on untouched pre-stroke cube packets."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from stationary_noise import observations, scatter, simulate, v

HERE = Path(__file__).resolve().parent


def quiet_packets(session):
    root = HERE.parent / "unit3-wheelstand" / session
    data = np.genfromtxt(root / "cube.csv.gz", delimiter=",", names=True)
    groups = data.reshape(-1, 3)
    assert np.all(groups["slot"] == [0, 1, 2])
    times = groups["sec"][:, 0]
    assert np.all(groups["sec"] == times[:, None])
    keep = np.zeros(len(times), dtype=bool)
    with (root / "actions.csv").open() as stream:
        for row in csv.DictReader(stream):
            if row["kind"] == "begin" and any(x in row["action"] for x in ("raise", "lower")):
                t = float(row["sec"])
                keep |= (times >= t - .8) & (times < t - .1)
    return np.stack([groups[k][keep] for k in "xyz"], axis=-1) / 8192


def predicted(densities, force):
    scalar = v["cube_accel_noise_density_ug"]
    packets = [simulate(scalar, force, seed, v["cube_accel_word_format"], np.array(densities)/scalar)
               for seed in range(4)]
    return scatter(np.concatenate(packets))


def main():
    fit = observations("resting")
    force = fit.mean(axis=(0, 1))
    scalar = v["cube_accel_noise_density_ug"]
    frozen = predicted([scalar] * 3, force)
    densities = np.rint(scalar * scatter(fit) / frozen)
    candidate = predicted(densities, force)
    rows = []
    fig, axes = plt.subplots(1, 3, sharey=True, figsize=(9, 3.6), layout="constrained")
    for ax, (label, packets) in zip(axes, [("Original rest", fit), ("D1", quiet_packets("D1")),
                                            ("D2", quiet_packets("D2"))]):
        real = scatter(packets)
        for i, axis in enumerate("XYZ"):
            rows.append(dict(session=label, packets=len(packets), axis=axis, density=densities[i],
                             real_mg=real[i], frozen_mg=frozen[i], candidate_mg=candidate[i]))
        for offset, values, name in [(-.25, real, "Recorded"), (0, frozen, "Frozen"), (.25, candidate, "Per axis")]:
            ax.bar(np.arange(3) + offset, values, .25, label=name)
        ax.set(title=f"{label} · {len(packets)} packets", xticks=range(3), xticklabels=list("XYZ"))
        print(label, real, "axis RMS errors", *(np.sqrt(np.mean((m-real)**2)) for m in [frozen,candidate]))
    axes[0].set_ylabel("Within-packet noise (mg RMS)")
    axes[-1].legend(fontsize=8)
    fig.savefig(HERE / "axis-noise.png", dpi=160)
    with (HERE / "axis-noise.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()

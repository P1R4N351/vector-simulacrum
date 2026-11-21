"""Resolve ground and fork load on the cube in the recorded pickup replay."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np

COLUMNS = ["shelf_z", "fork_shell_z", "fork_rubber_z", "link_z", "lift_xy"]


def contact_load(model, data, cube, floor, fork):
    """Return signed world-vertical loads and the horizontal lift resultant in N."""
    load = np.zeros(5)
    horizontal = np.zeros(2)
    wrench = np.empty(6)
    for index, contact in enumerate(data.contact):
        first, second = contact.geom
        if model.geom_bodyid[second] == cube:
            target, other, sign = second, first, 1
        elif model.geom_bodyid[first] == cube:
            target, other, sign = first, second, -1
        else:
            continue
        mujoco.mj_contactForce(model, data, index, wrench)
        force = sign * wrench[:3] @ contact.frame.reshape(3, 3)
        if other == floor:
            load[0] += force[2]
        else:
            horizontal += force[:2]
            group = 3
            if model.geom_bodyid[other] == fork:
                group = 2 if model.geom(target).name.startswith("cube_rubber_") else 1
            load[group] += force[2]
    load[4] = np.linalg.norm(horizontal)
    return load


def save(directory, rows, stem="pickup-support"):
    path = Path(directory) / f"{stem}.csv"
    with path.open("w") as output:
        writer = csv.writer(output, lineterminator="\n")
        writer.writerow(["initialization", "window_end_s", "lift_mm", *COLUMNS])
        writer.writerows(rows)
    fig, (ax, hand) = plt.subplots(2, 1, sharex=True, figsize=(8, 5.5), layout="constrained")
    data = np.array([r[1:] for r in rows if r[0] == "Last sighting + pitch"])
    t, lift, shelf, shell, rubber, link, horizontal = data.T
    ax.plot(t, shelf, label="Shelf")
    ax.plot(t, shell + rubber + link, label="Lift assembly")
    ax.axhline(1, color="gray", ls=":", label="Cube weight")
    ax.set_ylabel("Upward load / weight")
    hand.plot(t, horizontal, color="#955512", label="Horizontal lift force / weight")
    hand.set_ylabel("Horizontal force / weight")
    wrist = hand.twinx()
    wrist.plot(t, lift, color="#136883", label="Wrist height")
    wrist.set_ylabel("Recorded wrist height (mm)")
    ax.legend(fontsize=8)
    hand.legend(loc="upper left", fontsize=8)
    wrist.legend(loc="upper right", fontsize=8)
    hand.set_xlabel("Seconds after pickup command")
    ax.set_title("Where the cube's weight goes during the simulated pickup")
    fig.savefig(Path(directory) / f"{stem}.png", dpi=160)
    plt.close(fig)

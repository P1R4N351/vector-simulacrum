"""Sample the net contact force exerted by the printed fork on the free cube."""

from evaluate import run, here
import csv
import mujoco as mj
import numpy as np

if __name__ == "__main__":
    with (here / "support.csv").open("w") as f:
        out = csv.writer(f, lineterminator="\n")
        out.writerow(
            ["trial", "sec", "lift_mm", "contacts", "forward_N", "left_N", "up_N"]
        )
        for trial in range(1, 6):

            def observe(model, data, sec, lift):
                net = np.zeros(3)
                force = np.empty(6)
                fork = model.body("fork").id
                cube = model.body("cube").id
                count = 0
                for i, contact in enumerate(data.contact):
                    first, second = model.geom_bodyid[contact.geom]
                    if {first, second} != {fork, cube}:
                        continue
                    mj.mj_contactForce(model, data, i, force)
                    net += (1 if second == cube else -1) * (
                        force[:3] @ contact.frame.reshape(3, 3)
                    )
                    count += 1
                out.writerow(
                    [trial, round(sec, 4), round(lift, 3), count, *net.round(5)]
                )

            run((trial, "stl", "overhead", "weld", 0, 0, 0), observe)

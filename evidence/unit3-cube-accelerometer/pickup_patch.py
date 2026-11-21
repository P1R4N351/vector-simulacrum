"""Map the loaded fork contact patch in cube coordinates during acquisition."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np


def collect(model, data, cube, fork, time, dt, rows):
    wrench = np.empty(6)
    for index, contact in enumerate(data.contact):
        a, b = contact.geom
        if model.geom_bodyid[a] == fork and model.geom_bodyid[b] == cube:
            face, sign = b, 1
        elif model.geom_bodyid[b] == fork and model.geom_bodyid[a] == cube:
            face, sign = a, -1
        else:
            continue
        mujoco.mj_contactForce(model, data, index, wrench)
        if wrench[0] <= 0:
            continue
        basis = sign * contact.frame.reshape(3, 3)
        normal_z = basis[0, 2]
        position = 1000 * data.xmat[cube].reshape(3, 3).T @ (contact.pos - data.xpos[cube])
        rows.append((time, model.geom(face).name, *position, normal_z,
                     wrench[0]*dt, wrench[0]*normal_z*dt, wrench[1:3] @ basis[1:, 2]*dt))


def save(directory, rows):
    directory = Path(directory)
    with (directory / "pickup-patch.csv").open("w") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["time_s", "cube_geom", "x_mm", "y_mm", "z_mm", "normal_z",
                         "normal_impulse_Ns", "normal_up_Ns", "friction_up_Ns"])
        for t, geom, *values in rows:
            writer.writerow([round(t, 4), geom, *np.round(values, 9)])
    fig, axes = plt.subplots(1, 2, sharey=True, figsize=(9, 4), layout="constrained")
    for ax, (low, high) in zip(axes, [(2.17, 2.86), (2.86, 3.18)]):
        selected = [r for r in rows if low <= r[0] < high]
        x, y, z, nz, impulse, up, friction = np.array([r[2:] for r in selected]).T
        sizes = np.clip(impulse*3e4, 1, 100)
        dots = ax.scatter(np.maximum(abs(x), abs(y)), z, c=nz, vmin=-1, vmax=1,
                          cmap="coolwarm", s=sizes, alpha=.65, linewidths=0)
        ax.axhline(0, color="gray", lw=.7)
        ax.axhline(-12.55, color="#764127", ls=":", lw=1, label="Lower rim level")
        ax.legend(loc="upper left", fontsize=8)
        ax.set(xlim=(21.3, 22.3), ylim=(-23, 23), xlabel="max(|x|, |y|) in cube frame (mm)",
               title=f"{low:.2f}–{high:.2f} s")
        print(low, high, "normal/friction vertical impulse", up.sum(), friction.sum(),
              "above center", impulse[z>0].sum()/impulse.sum(),
              "up-pointing fraction", impulse[nz>.5].sum()/impulse.sum(),
              "range z", z.min(), z.max())
    axes[0].set_ylabel("Height above cube center (mm)")
    fig.colorbar(dots, ax=axes, label="Vertical component of contact normal")
    fig.suptitle("Simulated fork contacts on the cube · marker area scales with impulse")
    fig.savefig(directory / "pickup-patch.png", dpi=150)
    plt.close(fig)

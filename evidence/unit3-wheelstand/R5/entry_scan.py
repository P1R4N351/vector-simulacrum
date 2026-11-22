"""Locate the first mechanical stop along a horizontal, fixed-attitude approach."""
import csv
import itertools

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation


def scan(here, model, wrist, rails, rail_q, attitude, carry_offset):
    data = mj.MjData(model)
    data.mocap_pos[model.body("fork").mocapid[0]] = wrist
    for i in range(2):
        idx = model.body(f"rail{i}").mocapid[0]
        data.mocap_pos[idx], data.mocap_quat[idx] = rails[0, i], rail_q[0]
    cube = data.joint("cube_root").qpos
    rows = []
    for yaw, dz in itertools.product(range(0, 360, 45), range(-12, 13, 3)):
        cube[:3] = wrist + .001 * (np.asarray(carry_offset) + [0, 0, dz])
        cube[3:] = (Rotation.from_euler("z", yaw, degrees=True) * attitude).as_quat(scalar_first=True)
        nominal = cube[0]
        for offset in np.arange(50., -19., -.25):
            cube[0] = nominal + .001 * offset
            mj.mj_forward(model, data)
            if any(c.dist < 0 for c in data.contact):
                break
        else:
            raise ValueError("Scan ended before contact")
        assert offset < 50
        clear, blocked = offset + .25, offset
        for _ in range(8):
            mid = (clear + blocked) / 2
            cube[0] = nominal + .001 * mid
            mj.mj_forward(model, data)
            if any(c.dist < 0 for c in data.contact):
                blocked = mid
            else:
                clear = mid
        cube[0] = nominal + .001 * blocked
        mj.mj_forward(model, data)
        c = min(data.contact, key=lambda c: c.dist)
        a, b = (model.geom(int(i)).name for i in c.geom)
        assert a.startswith("cube_") or b.startswith("cube_")
        cube_first = a.startswith("cube_")
        normal = c.frame[:3] * (-1 if cube_first else 1)
        rows.append([yaw, dz, clear, blocked, a if cube_first else b,
                     b if cube_first else a, *np.round((c.pos - wrist) * 1000, 3), normal[2]])
    with (here / "entry-scan.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow("yaw_deg vertical_mm clear_dx_mm contact_dx_mm cube_geom robot_geom wrist_x_mm wrist_y_mm wrist_z_mm upward_normal".split())
        writer.writerows(rows)
    fig, axes = plt.subplots(1, 2, figsize=(9, 4), layout="constrained")
    for yaw in range(0, 360, 45):
        group = [r for r in rows if r[0] == yaw]
        axes[0].plot([r[1] for r in group], [r[3] for r in group], ".-", label=f"{yaw}°")
    axes[0].axhline(0, color=".4", ls=":")
    axes[0].legend(fontsize=7, ncol=2, title="Cube yaw")
    axes[0].set(xlabel="Vertical offset (mm)", ylabel="Forward offset at first contact (mm)")
    for i in range(model.ngeom):
        if model.geom_bodyid[i] != model.body("fork").id or model.geom_type[i] != mj.mjtGeom.mjGEOM_MESH:
            continue
        mesh = model.geom_dataid[i]
        first = model.mesh_vertadr[mesh]
        vertices = model.mesh_vert[first:first + model.mesh_vertnum[mesh]]
        world = vertices @ data.geom_xmat[i].reshape(3, 3).T + data.geom_xpos[i]
        front = ((world - wrist) * 1000)[:, 1:]
        boundary = front[ConvexHull(front).vertices]
        axes[1].fill(*boundary.T, color=".65", linewidth=0, zorder=0)
    axes[1].set_aspect("equal")
    for material, marker in [("shell", "o"), ("rubber", "x")]:
        group = [r for r in rows if material in r[4]]
        axes[1].scatter([r[7] for r in group], [r[8] for r in group], marker=marker, label=material, alpha=.65)
    axes[1].set(xlabel="Lateral position from wrist (mm)", ylabel="Height above wrist (mm)")
    axes[1].legend(title="Cube contact")
    fig.savefig(here / "entry-scan.png", dpi=155)

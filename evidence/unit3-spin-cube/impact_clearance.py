"""Signed clearance at the first physical 0.2 g packet, with the cube held at an observed pose."""

import csv
import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
import mount_replay as m
from earlier_inputs import TRIALS


def closest(model, data, parts, cube):
    # Use the solver's deepest actual contact point for intersecting pairs.
    overlaps = [
        (float(c.dist), int(c.geom1), int(c.geom2))
        for c in data.contact
        if c.dist < 0
        and (
            (c.geom1 in parts and c.geom2 in cube)
            or (c.geom2 in parts and c.geom1 in cube)
        )
    ]
    if overlaps:
        dist, a, b = min(overlaps)
        if a in cube:
            a, b = b, a
    else:
        dist, a, b = min(
            (mj.mj_geomDistance(model, data, a, b, 0.3, None), a, b)
            for a in parts
            for b in cube
        )
    return dist * 1000, model.geom(a).name, model.geom(b).name


def main():
    v, derived, xml = m._load_model_config()
    model = m.build_model(xml, v, derived, drive="mocap", contact="full")
    bodies = model.geom_bodyid
    groups = {"Fork": set(np.flatnonzero(bodies == model.body("fork").id))}
    chassis = set(np.flatnonzero(bodies == model.body("robot_chassis").id))
    groups["Chassis"] = {model.geom("chassis_geom").id}
    groups["Wheels and tread supports"] = chassis - groups["Chassis"]
    cubes = set(np.flatnonzero(bodies == model.body("cube").id))
    paths = m._load_structured("earlier-path.csv.gz")
    onsets = m._load_structured("impact-onset.csv")
    fig, ax = plt.subplots(figsize=(10, 4.5), layout="constrained")
    with (m.HERE / "impact-clearance.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            "trial placement pose_s physical_s group distance_mm robot_geom cube_geom".split()
        )
        for k, (label, file) in enumerate(
            [
                ("Initial", "impact-initial-poses.csv"),
                ("Last seen", "earlier-pose.csv.gz"),
            ]
        ):
            poses = m._load_structured(file)
            for j, name in enumerate(TRIALS):
                t = float(
                    onsets[
                        (onsets["trial"] == name)
                        & (onsets["phase_ms"] == 0)
                        & (onsets["level_g"] == 0.2)
                    ]["physical_s"][0]
                )
                p = poses[poses["trial"] == name][0]
                if p["t_s"] > t:
                    continue
                traj = m.compute_trajectory(
                    paths[paths["trial"] == name], v, model.opt.timestep
                )
                i = int(np.argmin(abs(traj["times"] - t)))
                data = mj.MjData(model)
                data.qpos[:7] = [
                    p["x_mm"] / 1000,
                    p["y_mm"] / 1000,
                    0.0227,
                    np.cos(p["yaw_rad"] / 2),
                    0,
                    0,
                    np.sin(p["yaw_rad"] / 2),
                ]
                for body, key in [
                    ("fork", "fork_pos"),
                    ("robot_chassis", "chassis_pos"),
                ]:
                    mid = int(model.body_mocapid[model.body(body).id])
                    data.mocap_pos[mid] = traj[key][i]
                    data.mocap_quat[mid] = traj["quat"][i]
                mj.mj_forward(model, data)
                for g, (group, parts) in enumerate(groups.items()):
                    dist, a, b = closest(model, data, parts, cubes)
                    w.writerow([name, label, p["t_s"], t, group, dist, a, b])
                    ax.scatter(
                        j + (g - 1) * 0.18 + 0.04 * k,
                        dist,
                        marker=["o", "x"][k],
                        color=["teal", "darkorange", "slateblue"][g],
                        label=f"{group}, {label}" if j == 0 else None,
                    )
    ax.axhline(0, color="black", lw=0.8)
    ax.set_xticks(range(len(TRIALS)), TRIALS)
    ax.set_ylabel("Signed clearance to fixed cube (mm)")
    ax.legend(ncol=3, fontsize=8)
    fig.savefig(m.HERE / "impact-clearance.png", dpi=160)


if __name__ == "__main__":
    main()

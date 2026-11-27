"""Compare W4 support forces with the physical and simulated cube's specific force."""

import csv
import json
import subprocess
import numpy as np
import matplotlib.pyplot as plt
import mujoco as mj
import mount_replay as m
from optical_mount import top_track
from front_contact import front_model


def main():
    cfg, der, src = m._load_model_config()
    paths = m._load_structured("earlier-path.csv.gz")
    slots = m._load_structured("impact-samples.csv.gz")
    slots = slots[
        (slots["trial"] == "W4")
        & (slots["packet_s"] > -0.8)
        & (slots["packet_s"] < 0.1)
    ]
    pt = slots["packet_s"]
    pn = np.linalg.norm(np.column_stack([slots[k] for k in "xyz"]), axis=1) / 8192
    fig, axes = plt.subplots(
        4, 2, figsize=(11, 8), sharex=True, sharey=True, layout="constrained"
    )
    with (m.HERE / "support-trace.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            "placement mode start_s end_s duration_ms robot_contact_fraction longest_unsupported_ms fork_vertical_impulse_Ns chassis_vertical_impulse_Ns physical_median_g simulated_median_g physical_packets physical_slots_below_0_3g physical_min_g simulated_slots simulated_slots_below_0_3g simulated_min_g".split()
        )
        for j, (label, file) in enumerate(
            [
                ("Initial", "impact-initial-poses.csv"),
                ("Last seen", "earlier-pose.csv.gz"),
            ]
        ):
            poses = m._load_structured(file)
            pose = poses[poses["trial"] == "W4"][0]
            for row, mode in enumerate(("fork", "front4", "front16", "full")):
                ax = axes[row, j]
                model = (
                    front_model(int(mode[5:]))[0]
                    if mode.startswith("front")
                    else m.build_model(src, cfg, der, drive="freejoint", contact=mode)
                )
                traj = m.compute_trajectory(
                    paths[paths["trial"] == "W4"], cfg, model.opt.timestep
                )
                cube = model.body("cube").id
                loads = []
                forces = []

                def observe(data):
                    shelf = robot = fork_z = chassis_z = 0.0
                    wrench = np.zeros(6)
                    for ci, c in enumerate(data.contact):
                        b1, b2 = model.geom_bodyid[c.geom]
                        if cube not in (b1, b2) or c.efc_address < 0:
                            continue
                        mj.mj_contactForce(model, data, ci, wrench)
                        if 0 in (b1, b2):
                            shelf += wrench[0]
                        else:
                            robot += wrench[0]
                            z_force = (c.frame.reshape(3, 3).T @ wrench[:3])[2] * (
                                1 if b2 == cube else -1
                            )
                            other = b1 if b2 == cube else b2
                            if other == model.body("fork").id:
                                fork_z += z_force
                            else:
                                chassis_z += z_force
                    loads.append((shelf, robot, fork_z, chassis_z))
                    forces.append((data.sensor("cube_accel").data / 9.81).tolist())

                top_track(model, traj, pose, observe)
                dt = model.opt.timestep
                times = traj["times"] + dt
                shelf, robot, fork_z, chassis_z = np.asarray(loads).T
                args = dict(dt=dt, start=traj["times"][0], forces=forces)
                arr = np.array(
                    json.loads(
                        subprocess.check_output(
                            ["node", str(m.HERE / "mount_sensor.mjs")],
                            input=json.dumps(args),
                            text=True,
                        )
                    )
                )
                st = arr[:, 0]
                sn = np.linalg.norm(arr[:, 1:4], axis=1)
                # Record every sustained loss of floor load, then inspect robot load within it.
                no_load = (shelf < 1e-5) & (times >= -0.8) & (times <= 0.1)
                edges = np.flatnonzero(np.diff(np.r_[False, no_load, False]))
                for a, b in edges.reshape(-1, 2):
                    start, end = times[a] - dt, times[b - 1]
                    if end - start < 0.02:
                        continue
                    ax.axvspan(start, end, color="gold", alpha=0.4)
                    ph = (pt >= start) & (pt < end)
                    si = (st >= start) & (st < end)
                    unsupported = robot[a:b] < 1e-5
                    intervals = np.flatnonzero(
                        np.diff(np.r_[False, unsupported, False])
                    ).reshape(-1, 2)
                    longest = (
                        max((hi - lo for lo, hi in intervals), default=0) * dt * 1000
                    )
                    w.writerow(
                        [
                            label,
                            mode,
                            start,
                            end,
                            round(1000 * (end - start)),
                            1 - np.mean(unsupported),
                            longest,
                            dt * sum(fork_z[a:b]),
                            dt * sum(chassis_z[a:b]),
                            np.median(pn[ph]) if ph.any() else "",
                            np.median(sn[si]) if si.any() else "",
                            len(set(pt[ph])),
                            sum(pn[ph] < 0.3),
                            min(pn[ph], default=""),
                            sum(si),
                            sum(sn[si] < 0.3),
                            min(sn[si], default=""),
                        ]
                    )
                keep = (st >= -0.8) & (st < 0.1)
                ax.scatter(st[keep], sn[keep], s=7, color="teal", label="Simulated")
                ax.scatter(pt, pn, s=8, color="black", label="Physical")
                ax.set_title(f"{label} placement · {mode}")
                ax.set_ylim(-0.1, 5.2)
    axes[0, 0].legend(fontsize=8)
    fig.supylabel("Cube accelerometer magnitude (g)")
    fig.supxlabel(
        "Seconds from pitch crossing · gold marks simulated loss of shelf contact ≥20 ms"
    )
    fig.savefig(m.HERE / "support-trace.png", dpi=160)


if __name__ == "__main__":
    main()

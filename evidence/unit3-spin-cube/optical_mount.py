"""Compare W4 top glyph travel to replayed horizontal motion of the cube's top center."""

import csv
import numpy as np
import matplotlib.pyplot as plt
import mujoco as mj
import mount_replay as m
from front_contact import front_model

# Difference of host SDK event times: first pitch sample above 45° minus wheelstand command.
PITCH_AFTER_COMMAND = 4.2768703


def top_track(model, traj, pose, observe_step=None):
    data = mj.MjData(model)
    cube = model.body("cube").id
    adr = int(model.jnt_qposadr[int(model.body_jntadr[cube])])
    q = pose["yaw_rad"] / 2
    data.qpos[adr : adr + 7] = [
        pose["x_mm"] / 1000,
        pose["y_mm"] / 1000,
        0.0227,
        np.cos(q),
        0,
        0,
        np.sin(q),
    ]
    targets = m._driven_targets(model, traj, "freejoint")
    buffers = (model.qpos0.copy(), model.qpos0.copy(), np.zeros(model.nv))
    dt = model.opt.timestep
    for _ in range(250):
        m._apply_drive(model, data, "freejoint", targets, buffers, 0, 0, dt)
        mj.mj_step(model, data)

    def point():
        return (data.xpos[cube] + 0.022 * data.xmat[cube].reshape(3, 3)[:, 2]).copy()

    mj.mj_forward(model, data)
    origin = point()
    points = []
    for i in range(len(traj["times"])):
        m._apply_drive(
            model,
            data,
            "freejoint",
            targets,
            buffers,
            i,
            min(i + 1, len(traj["times"]) - 1),
            dt,
        )
        mj.mj_step(model, data)
        if observe_step is not None:
            observe_step(data)
        # Update transforms to the post-integration pose before reading the top point.
        mj.mj_kinematics(model, data)
        tilt = np.degrees(np.arccos(np.clip(data.xmat[cube, 8], -1, 1)))
        points.append([*((point() - origin) * 1000), tilt])
    return np.asarray(points)


def main():
    cfg, derived, src = m._load_model_config()
    camera = np.genfromtxt(
        m.HERE.parent / "unit3-wheelstand/W4/optical-travel.csv",
        delimiter=",",
        names=True,
    )
    paths = m._load_structured("earlier-path.csv.gz")
    fig, axes = plt.subplots(
        2, 2, figsize=(11, 6), sharex=True, sharey="row", layout="constrained"
    )
    with (m.HERE / "optical-mount.csv").open("w") as out:
        writer = csv.writer(out, lineterminator="\n")
        writer.writerow(
            "placement contact t_s optical_mm correlation sim_top_xy_mm sim_top_z_change_mm sim_top_tilt_deg".split()
        )
        for ax, (label, file) in zip(
            axes[0],
            [
                ("Initial", "impact-initial-poses.csv"),
                ("Last seen", "earlier-pose.csv.gz"),
            ],
        ):
            poses = m._load_structured(file)
            pose = poses[poses["trial"] == "W4"][0]
            for mode in ("fork", "front4", "front16", "full"):
                model = (
                    front_model(int(mode[5:]))[0]
                    if mode.startswith("front")
                    else m.build_model(
                        src, cfg, derived, drive="freejoint", contact=mode
                    )
                )
                traj = m.compute_trajectory(
                    paths[paths["trial"] == "W4"], cfg, model.opt.timestep
                )
                xyz = top_track(model, traj, pose)
                t = traj["times"] + model.opt.timestep
                dist = np.linalg.norm(xyz[:, :2], axis=1)
                times = camera["s_after_command"] - PITCH_AFTER_COMMAND
                keep = (times >= t[0]) & (times <= t[-1])
                ax.plot(t, dist, label=mode)
                lower = axes[1, 0 if label == "Initial" else 1]
                lower.plot(t, xyz[:, 2], label=mode)
                lower.set_xlabel("Seconds from first pitch above 45°")
                for c, ts in zip(camera[keep], times[keep]):
                    writer.writerow(
                        [
                            label,
                            mode,
                            ts,
                            c["travel_mm"],
                            c["correlation"],
                            np.interp(ts, t, dist),
                            np.interp(ts, t, xyz[:, 2]),
                            np.interp(ts, t, xyz[:, 3]),
                        ]
                    )
            ax.scatter(
                times[keep],
                camera["travel_mm"][keep],
                s=20,
                color="black",
                label="Overhead glyph",
                zorder=5,
            )
            ax.set(
                title=f"{label} cube pose", xlabel="Seconds from first pitch above 45°"
            )
        axes[1, 0].set_ylabel("Simulated top height change (mm)")
        axes[0, 0].set_ylabel("Top center displacement in shelf plane (mm)")
        axes[0, 1].legend(fontsize=8)
    fig.savefig(m.HERE / "optical-mount.png", dpi=160)


if __name__ == "__main__":
    main()

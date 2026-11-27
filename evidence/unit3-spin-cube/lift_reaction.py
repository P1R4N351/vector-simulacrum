"""Estimate contact-only generalized lift load under the replay's leveled-fork linkage."""

import csv
import numpy as np
import matplotlib.pyplot as plt
import mujoco as mj
import mount_replay as m
from earlier_inputs import TRIALS
from front_contact import front_model
from optical_mount import top_track


def main():
    model, v = front_model(4)
    cap = v["lift_servo_torque_max_mNm"]
    path = m._load_structured("earlier-path.csv.gz")
    initial = m._load_structured("impact-initial-poses.csv")
    last = m._load_structured("earlier-pose.csv.gz")
    cube = model.body("cube").id
    fork = model.body("fork").id
    fig, axes = plt.subplots(
        2, 4, figsize=(12, 5.8), sharex=True, sharey=True, layout="constrained"
    )
    axes.flat[-1].set_visible(False)
    with (m.HERE / "lift-reaction.csv").open("w") as out:
        w = csv.writer(out, lineterminator="\n")
        w.writerow(
            "trial pose peak_abs_mNm ms_over_cap longest_over_cap_ms peak_s".split()
        )
        for k, trial in enumerate(TRIALS):
            ax = axes.flat[k]
            robot = path[path["trial"] == trial]
            traj = m.compute_trajectory(robot, v, model.opt.timestep)
            t = traj["times"]
            height = np.clip(
                np.interp(t, robot["t_s"], robot["lift_mm"]),
                v["lift_height_min_mm"],
                v["lift_height_max_mm"],
            )
            # Per radian of lift rotation, the leveled fork translates by
            # (-L sin(angle), 0, L cos(angle)) in the chassis frame.
            dz = height - v["lift_base_position_mm"][2]
            reach = np.sqrt(v["lift_arm_length_mm"] ** 2 - dz**2)
            local = np.stack([-dz, np.zeros_like(dz), reach], axis=1) / 1000
            for label, poses in [("Initial", initial), ("Last seen", last)]:
                pose = poses[poses["trial"] == trial][0]
                if (
                    label == "Last seen"
                    and pose["t_s"] == initial[initial["trial"] == trial][0]["t_s"]
                ):
                    continue
                load = []

                def observe(data):
                    world_force = np.zeros(3)
                    wrench = np.empty(6)
                    for i, c in enumerate(data.contact):
                        a, b = model.geom_bodyid[c.geom]
                        if {a, b} != {fork, cube} or c.efc_address < 0:
                            continue
                        mj.mj_contactForce(model, data, i, wrench)
                        world_force += (1 if b == fork else -1) * (
                            c.frame.reshape(3, 3).T @ wrench[:3]
                        )
                    j = data.xmat[fork].reshape(3, 3) @ local[len(load)]
                    load.append(1000 * np.dot(world_force, j))

                top_track(model, traj, pose, observe)
                win = (t >= -0.8) & (t <= 0.1)
                tw = t[win]
                tau = np.asarray(load)[win]
                above = abs(tau) > cap
                edges = np.flatnonzero(np.diff(np.r_[False, above, False])).reshape(
                    -1, 2
                )
                dt_ms = model.opt.timestep * 1000
                peak = np.argmax(abs(tau))
                w.writerow(
                    [
                        trial,
                        label,
                        abs(tau[peak]),
                        dt_ms * above.sum(),
                        dt_ms * max((b - a for a, b in edges), default=0),
                        tw[peak],
                    ]
                )
                ax.plot(tw, tau, label=label, lw=1)
            ax.axhspan(-cap, cap, color="grey", alpha=0.15)
            ax.set(title=trial, yscale="symlog")
            ax.set_yscale("symlog", linthresh=60)
    axes.flat[0].legend(fontsize=8)
    fig.supylabel("Cube reaction on lift (mN·m)")
    fig.supxlabel("Seconds from first SDK pitch between 45° and 85°")
    fig.savefig(m.HERE / "lift-reaction.png", dpi=160)


if __name__ == "__main__":
    main()

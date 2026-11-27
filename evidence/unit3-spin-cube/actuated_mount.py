"""Drive chassis pose while the canonical torque-limited lift tracks measured height."""

import csv
import json
import subprocess
import xml.etree.ElementTree as ET
import mujoco as mj
import numpy as np
from earlier_inputs import TRIALS
from tap_timing import events
import mount_replay as m


def build(v, derived, src, spinning=False, welded_supports=False):
    scene = ET.Element("mujoco")
    ET.SubElement(scene, "compiler", angle="radian")
    scene.append(src.find("option"))
    world = ET.SubElement(scene, "worldbody")
    ET.SubElement(
        world, "geom", type="plane", size="1 1 .01", contype="0", conaffinity="1"
    )
    body = ET.SubElement(world, "body", name="robot_chassis")
    m._attach_drive(
        body,
        "robot_chassis",
        "freejoint",
        v["total_mass_g"] / 1000,
        derived["chassis_inertia"],
    )
    for side in ("l", "r"):
        geom = src.find(f".//geom[@name='tip_front_{side}']")
        if spinning or welded_supports:
            # Axle runs along chassis +Y. Keep identical cylinder endpoints in the chassis frame.
            center = (
                np.array([float(n) for n in geom.get("fromto").split()])
                .reshape(2, 3)
                .mean(axis=0)
            )
            carrier = ET.SubElement(body, "body", name=f"rotor_{side}")
            wheel = src.find(f".//joint[@name='wheel_f{side}_joint']")
            if spinning:
                ET.SubElement(
                    carrier,
                    "joint",
                    name=f"spin_{side}",
                    type="hinge",
                    pos=" ".join(map(str, center)),
                    axis="0 1 0",
                    armature=wheel.get("armature"),
                )
            ET.SubElement(
                carrier,
                "inertial",
                pos=" ".join(map(str, center)),
                mass="0.002",
                diaginertia="1e-7 2e-7 1e-7",
            )
            carrier.append(geom)
        else:
            body.append(geom)
    body.append(src.find(".//body[@name='lift_arm']"))
    world.append(src.find(".//body[@name='cube']"))
    ET.SubElement(scene, "equality").append(src.find(".//joint[@name='lift_level']"))
    ET.SubElement(scene, "actuator").append(
        src.find("./actuator/position[@name='lift']")
    )
    ET.SubElement(
        ET.SubElement(scene, "sensor"),
        "accelerometer",
        name="cube_accel",
        site="cube_chip",
    )
    return mj.MjModel.from_xml_string(ET.tostring(scene, encoding="unicode"))


def replay(
    output,
    feedforward=False,
    unloaded=False,
    trajectories=None,
    path_override=None,
    trials=TRIALS,
    packet_observer=None,
    wheel_inputs=None,
    welded_supports=False,
):
    v, d, s = m._load_model_config()
    model = build(
        v, d, s, spinning=wheel_inputs is not None, welded_supports=welded_supports
    )
    if unloaded:
        # Keep gravity and linkage dynamics but disable all cube contacts.
        cube_geoms = model.geom_bodyid == model.body("cube").id
        model.geom_contype[cube_geoms] = 0
        model.geom_conaffinity[cube_geoms] = 0
    dt = model.opt.timestep
    path, poses, packets = [
        m._load_structured(f"earlier-{k}.csv.gz") for k in ("path", "pose", "packets")
    ]
    if path_override is not None:
        path = path_override
    with output.open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(
            "trial phase_ms physical_count simulated_count first_error_ms lift_height_rmse_mm peak_torque_mNm".split()
        )
        for name in trials:
            robot = path[path["trial"] == name]
            pose = poses[poses["trial"] == name][0]
            packets_trial = packets[packets["trial"] == name]
            real = events(packets_trial["t_s"], packets_trial["tap"])
            traj = m.compute_trajectory(robot, v, dt)
            t = traj["times"]
            h = np.interp(t, robot["t_s"], robot["lift_mm"])
            length = v["lift_arm_length_mm"]
            pivot_z = v["lift_base_position_mm"][2]
            angle = np.arcsin(np.clip((h - pivot_z) / length, -1, 1))
            data = mj.MjData(model)
            q = pose["yaw_rad"] / 2
            data.joint("cube_root").qpos[:] = [
                pose["x_mm"] / 1000,
                pose["y_mm"] / 1000,
                0.0227,
                np.cos(q),
                0,
                0,
                np.sin(q),
            ]
            data.ctrl[0] = angle[0]
            data.joint("lift_joint").qpos[0] = angle[0]
            data.joint("wrist_joint").qpos[0] = -angle[0]
            tg = m._driven_targets(model, traj, "freejoint")
            buf = (model.qpos0.copy(), model.qpos0.copy(), np.zeros(model.nv))
            for _ in range(250):
                m._apply_drive(model, data, "freejoint", tg, buf, 0, 0, dt)
                mj.mj_step(model, data)
            heights = []
            forces = []
            torque = []
            target = angle
            if feedforward:
                lead_s = (
                    v["lift_servo_kv_mNms_per_rad"] / v["lift_servo_kp_mNm_per_rad"]
                )
                # Backward difference of the interpolated trajectory for offline replay.
                velocity = np.diff(angle, prepend=angle[0]) / dt
                target = np.clip(
                    angle + lead_s * velocity, *model.actuator_ctrlrange[0]
                )
            if wheel_inputs is not None:
                wheel_rows = wheel_inputs[wheel_inputs["trial"] == name]
                assert np.array_equal(wheel_rows["t_s"], robot["t_s"])
                rates = []
                for side, field in (("l", "left_mmps"), ("r", "right_mmps")):
                    radius_mm = 1000 * model.geom(f"tip_front_{side}").size[0]
                    rates.append(
                        np.interp(t, wheel_rows["t_s"], wheel_rows[field]) / radius_mm
                    )
            for i, cmd in enumerate(target):
                if wheel_inputs is not None:
                    for side, rate in zip(("l", "r"), rates):
                        joint = data.joint(f"spin_{side}")
                        joint.qpos[0] = 0  # rotationally symmetric surface
                        joint.qvel[0] = rate[i]
                m._apply_drive(
                    model, data, "freejoint", tg, buf, i, min(i + 1, len(t) - 1), dt
                )
                data.ctrl[0] = cmd
                mj.mj_step(model, data)
                heights.append(
                    pivot_z + length * np.sin(data.joint("lift_joint").qpos[0])
                )
                torque.append(abs(data.actuator_force[0]) * 1000)
                forces.append((data.sensor("cube_accel").data / 9.81).tolist())
            win = (t >= -0.8) & (t <= 0.1)
            err = np.sqrt(np.mean((np.array(heights)[win] - h[win]) ** 2))
            if trajectories is not None:
                trajectories[name] = (t + dt, h, np.array(heights))
            for phase in [] if unloaded else range(0, 10, 2):
                i = round(phase / (1000 * dt))
                arg = dict(dt=dt, start=t[i], forces=forces[i:])
                arr = np.array(
                    json.loads(
                        subprocess.check_output(
                            ["node", str(m.HERE / "mount_sensor.mjs")],
                            input=json.dumps(arg),
                            text=True,
                        )
                    )
                )
                if packet_observer is not None:
                    packet_observer(name, phase, arr)
                est = events(arr[:, 0], arr[:, 4])
                w.writerow(
                    [
                        name,
                        phase,
                        len(real),
                        len(est),
                        (est[0] - real[0]) * 1000 if len(est) else "",
                        err,
                        max(np.array(torque)[win]),
                    ]
                )


if __name__ == "__main__":
    replay(m.HERE / "actuated-mount.csv")

"""Record mirrored belly-side righting attempts with free bodies and joint actuators."""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from simulacrum import mjcf, replay
from light_cube import add_cube, collision_boxes


def patient_copy(element):
    patient = copy.deepcopy(element)
    for node in patient.iter():
        for key in ("name", "joint", "joint1", "joint2"):
            if key in node.attrib:
                node.set(key, "patient_" + node.get(key))
    return patient


def scene_xml(boxes=None, mass_g=50, friction=0.5, fixed_grip=False, corner_friction=0.9):
    root = ET.fromstring(
        mjcf.build_mjcf(dict(replay.nominal_values(), **replay.OPEN_TABLE))
    )
    world = root.find("worldbody")
    rescuer = world.find("body[@name='chassis']")
    patient = patient_copy(rescuer)
    world.append(patient)
    for geom in world.findall("geom"):
        geom.set("conaffinity", "3")
    # Separate collision groups enable inter-robot and table contact without self-contact.
    for body, group, other in ((rescuer, "1", "2"), (patient, "2", "1")):
        for geom in body.iter("geom"):
            if geom.get("contype", "1") != "0":
                geom.set("contype", group)
                geom.set("conaffinity", other)
    for section in ("equality", "actuator"):
        parent = root.find(section)
        parent.extend([patient_copy(entry) for entry in parent])
    if boxes:
        for geom in world.findall("geom"):
            geom.set("conaffinity", "7")
        add_cube(world, boxes, mass_g, friction, corner_friction)
        if fixed_grip:
            # Exact rigid mount at the stock carry transform. Cube inertia is retained.
            cube = world.find("body[@name='cube']")
            cube.remove(cube.find("freejoint"))
            world.remove(cube)
            patient.find(".//body[@name='patient_lift_fork']").append(cube)
            cube.set("pos", "0.026 0 -0.0125")
    return ET.tostring(root, encoding="unicode")


def axis_rotation(axis, angle):
    quat = np.zeros(4)
    mj.mju_axisAngle2Quat(quat, np.array(axis, dtype=float), angle)
    return quat


def pose(data, prefix):
    body = data.body(prefix + "chassis")
    joints = ("head", "lift", "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")
    return [
        *body.xpos,
        *body.xquat,
        *[data.joint(prefix + j + "_joint").qpos[0] for j in joints],
    ]


def run(model, side, args):
    data = mj.MjData(model)
    low, high = model.actuator("lift").ctrlrange
    root = data.joint("patient_root").qpos
    root[:3] = (0, 0, 0.039)
    mj.mju_mulQuat(
        root[3:],
        axis_rotation([0, 0, 1], side * np.pi / 2),
        axis_rotation([1, 0, 0], side * np.pi / 2),
    )
    data.joint("root").qpos[:3] = (-0.176, side * args.offset_mm * 0.001, 0.001)
    for prefix in ("", "patient_"):
        data.joint(prefix + "lift_joint").qpos[0] = low
        data.joint(prefix + "wrist_joint").qpos[0] = -low
        data.actuator(prefix + "lift").ctrl[0] = low

    has_cube = args.cube_obj is not None
    if has_cube:
        # Present a free cube between the sideways fork and the adjacent approach.
        # Use the settled side-rest height, so it is not dropped from above the fork.
        root[2] = 0.028
        mj.mj_forward(model, data)
        fork_center = data.geom("patient_lift_fork_geom").xpos.copy()
        half_depth = model.geom("patient_lift_fork_geom").size[0]
        if args.fixed_grip:
            cube_pos = data.body("cube").xpos
        else:
            cube_pos = data.joint("cube_root").qpos
            cube_pos[:3] = [
                fork_center[0],
                fork_center[1] + side * (0.022 + half_depth + 0.0005),
                0.0221,
            ]
            if args.presentation == "carry-pose":
                wrist = data.body("patient_lift_fork")
                cube_pos[:3] = wrist.xpos + wrist.xmat.reshape(3, 3) @ np.array(
                    [0.026, 0, -0.0125]
                )
                cube_pos[3:] = wrist.xquat
        data.joint("root").qpos[1] = cube_pos[1] + side * args.offset_mm * 0.001
    if args.cube_control:
        root[0] += 0.45
    if has_cube:
        values = replay.nominal_values()
        reverse_angle = mjcf.lift_angle_of_height(values, 68.0)
        reverse_start = None
        lower_start = None
        # Engage from above as in DA_ROLL_LOW.
        data.joint("lift_joint").qpos[0] = high
        data.joint("wrist_joint").qpos[0] = -high
    mj.mj_forward(model, data)
    initial_overlap_mm = 0.0
    if has_cube:
        for contact in data.contact:
            names = [model.geom(i).name for i in contact.geom]
            if any(n.startswith("cube_") for n in names) and any(
                n.startswith("patient_") for n in names
            ):
                initial_overlap_mm = max(initial_overlap_mm, -contact.dist * 1000)
    if args.predocked:
        # Let the free patient/fork assembly settle under gravity before placing the
        # rescuer at the known adjacent-face dock pose, with its fork above the cube.
        data.actuator("lift").ctrl[0] = high
        for _ in range(round(1.0 / model.opt.timestep)):
            mj.mj_step(model, data)
        cube_center = data.body("cube").xpos.copy()
        # Near face is along world -x; use its actual projected extent after settling.
        corners = np.array(
            [
                [a, b, c]
                for a in (-0.022, 0.022)
                for b in (-0.022, 0.022)
                for c in (-0.022, 0.022)
            ]
        )
        near_x = cube_center[0] + min(
            (corners @ data.body("cube").xmat.reshape(3, 3).T)[:, 0]
        )
        root_r = data.joint("root").qpos
        root_r[:2] = [near_x - 0.013, cube_center[1] + side * args.offset_mm * 0.001]
        mj.mj_forward(model, data)
        # Move back only enough to remove inter-robot overlap at initialization.
        for _ in range(100):
            overlap = False
            for c in data.contact:
                names = [model.geom(i).name for i in c.geom]
                is_target = [n.startswith(("patient_", "cube_")) for n in names]
                if (
                    any(is_target)
                    and not all(is_target)
                    and not any(n in ("table", "floor") for n in names)
                    and c.dist < -0.00005
                ):
                    overlap = True
                    break
            if not overlap:
                break
            root_r[0] -= 0.001
            mj.mj_forward(model, data)
        else:
            raise RuntimeError("could not place rescuer without penetration")
        data.time = 0
    cube_at_start = data.body("cube").xquat.copy() if has_cube else None
    simultaneous_steps = 0
    patient_fork_steps = 0
    peak_cube_tilt = 0.0
    first = None
    lift_start = None
    frames = []
    start_xy = None
    best_upright = 180.0
    worst_tilt = 0.0
    max_grip_error = 0.0
    contact_duration = 0.0
    for tick in range(
        round((10 if args.predocked else 7 if has_cube else 4.5) / model.opt.timestep)
    ):
        t = data.time
        if first and lift_start is None and t > first["t"] + args.delay_ms * 0.001:
            lift_start = t
        elapsed = t - lift_start if lift_start is not None else -1
        drive = args.power
        if t < 0.35 or elapsed >= 1.22:
            drive = 0
        elif elapsed >= 0.65:
            drive = -0.23
        if has_cube:
            # Small patient lift sweep before the rescuer arrives; freely slipping contact.
            data.actuator("patient_lift").ctrl[0] = low + np.radians(
                args.patient_raise_deg
            ) * np.clip((t - 0.4) / 0.5, 0, 1)
            x = data.joint("root").qpos[0]
            if lower_start is None and (
                (args.predocked and t >= 0.35)
                or (
                    not args.predocked
                    and x >= data.body("cube").xpos[0] - 0.022 - 0.013
                )
            ):
                lower_start = t
            angle = (
                high
                if lower_start is None
                else max(low, high - 0.75 * (t - lower_start))
            )
            if (
                lower_start is not None
                and reverse_start is None
                and data.joint("lift_joint").qpos[0] <= reverse_angle
            ):
                reverse_start = t
            speed = 0 if t < 0.35 else (50 if lower_start is None else 0)
            if reverse_start is not None and t - reverse_start < 1.5:
                speed = -60
            # Speed feedback and static-friction feedforward for the replay approach.
            measured = (
                data.joint("wheel_fl_joint").qvel[0] * values["wheel_front_radius_mm"]
            )
            drive = np.clip(
                np.sign(speed) * values["tread_friction_power_fraction"]
                + (speed + 0.6 * (speed - measured))
                / values["tread_noload_speed_mmps"],
                -1,
                1,
            )
        for name in ("left_wheel", "right_wheel"):
            data.actuator(name).ctrl[0] = drive
        rise = np.clip(elapsed / 0.48, 0, 1)
        lower = np.clip((1.65 - elapsed) / 0.38, 0, 1)
        data.actuator("lift").ctrl[0] = (
            angle if has_cube else low + (high - low) * rise * lower
        )
        mj.mj_step(model, data)
        point = []
        patient_on_cube = rescuer_on_target = False
        for contact in data.contact:
            names = [model.geom(i).name for i in contact.geom]
            cube_contact = any(n.startswith("cube_") for n in names)
            patient_contact = any(n.startswith("patient_") for n in names)
            rescuer_fork = any(
                n.startswith(("lift_fork_geom", "fork_side_")) for n in names
            )
            if (
                has_cube
                and cube_contact
                and any(
                    n.startswith(("patient_lift_fork_geom", "patient_fork_side_"))
                    for n in names
                )
            ):
                patient_on_cube = True
            if rescuer_fork and (cube_contact if has_cube else patient_contact):
                rescuer_on_target = True
                if not point:
                    point = [contact.pos.tolist()]
                if first is None:
                    first = dict(t=t, geoms=names, xyz_m=point[0])
        if first is not None:
            patient_body = data.body("patient_chassis")
            tilt = float(np.degrees(np.arccos(np.clip(patient_body.xmat[8], -1, 1))))
            best_upright = min(best_upright, tilt)
            worst_tilt = max(worst_tilt, tilt)
            if start_xy is None:
                start_xy = patient_body.xpos[:2].copy()
        contact_duration += rescuer_on_target * model.opt.timestep
        if args.fixed_grip:
            wrist = data.body("patient_lift_fork")
            error = (
                data.body("cube").xpos
                - wrist.xpos
                - wrist.xmat.reshape(3, 3) @ np.array([0.026, 0, -0.0125])
            )
            max_grip_error = max(max_grip_error, np.linalg.norm(error))
        simultaneous_steps += int(patient_on_cube and rescuer_on_target)
        patient_fork_steps += int(patient_on_cube)
        if has_cube:
            cube = data.body("cube")
            peak_cube_tilt = max(
                peak_cube_tilt, np.degrees(np.arccos(np.clip(cube.xmat[8], -1, 1)))
            )
        if tick % max(1, round(1 / 30 / model.opt.timestep)) == 0:
            frames.append(
                dict(t=t, contact=point, poses=[pose(data, ""), pose(data, "patient_")])
            )
            if has_cube:
                frames[-1]["cube"] = [*cube.xpos, *cube.xquat]
    vertical = data.body("patient_chassis").xmat.reshape(3, 3)[2, 2]
    summary = dict(
        first_fork_contact=first,
        lift_start_s=lift_start,
        final_patient_tilt_deg=float(np.degrees(np.arccos(np.clip(vertical, -1, 1)))),
    )
    if has_cube:
        summary.pop("lift_start_s")
        summary.update(
            rescuer_cube_contact_s=contact_duration,
            closest_to_upright_deg=best_upright if first else None,
            farthest_from_upright_deg=worst_tilt if first else None,
            patient_travel_after_contact_mm=float(
                np.linalg.norm(data.body("patient_chassis").xpos[:2] - start_xy) * 1000
            )
            if first
            else None,
        )
        if args.fixed_grip:
            summary["max_fixed_grip_position_error_mm"] = float(max_grip_error * 1000)
        summary.update(
            initial_cube_patient_overlap_mm=initial_overlap_mm,
            final_cube_tilt_deg=float(
                np.degrees(np.arccos(np.clip(data.body("cube").xmat[8], -1, 1)))
            ),
            net_cube_rotation_deg=float(
                np.degrees(
                    2
                    * np.arccos(
                        np.clip(
                            abs(np.dot(cube_at_start, data.body("cube").xquat)), 0, 1
                        )
                    )
                )
            ),
            lower_start_s=lower_start,
            reverse_start_s=reverse_start,
            both_forks_on_cube_s=simultaneous_steps * model.opt.timestep,
            patient_fork_on_cube_s=patient_fork_steps * model.opt.timestep,
            peak_cube_tilt_deg=peak_cube_tilt,
        )
    return frames, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="new directory for this run")
    parser.add_argument(
        "--offset-mm",
        type=float,
        default=0,
        help="lateral approach offset, mirrored between sides",
    )
    parser.add_argument(
        "--power", type=float, default=0.45, help="forward wheel duty (0 to 1)"
    )
    parser.add_argument(
        "--delay-ms",
        type=float,
        default=200,
        help="delay between first fork contact and lift",
    )
    parser.add_argument(
        "--cube-obj",
        type=Path,
        help="proffer the SDK cube and approach its adjacent face",
    )
    parser.add_argument("--patient-fork-stl", type=Path,
                        help="use Dauler's forks7.stl for the printed patient's contact surfaces (requires coacd)")
    parser.add_argument("--cube-mass-g", type=float, default=50)
    parser.add_argument("--corner-friction", type=float, default=0.9)
    parser.add_argument("--uniform-cube", action="store_true", help="Replay the earlier single-material geometry")
    parser.add_argument("--cube-friction", type=float, default=0.5)
    parser.add_argument(
        "--cube-control",
        action="store_true",
        help="move the patient clear to test the same cube roll alone",
    )
    parser.add_argument(
        "--presentation", choices=("beside-fork", "carry-pose"), default="beside-fork"
    )
    parser.add_argument(
        "--patient-raise-deg",
        type=float,
        default=0,
        help="sideways patient lift stroke before rescuer arrival",
    )
    parser.add_argument(
        "--fixed-grip",
        action="store_true",
        help="rigidly attach cube mass to the articulated patient fork at the carry pose",
    )
    parser.add_argument(
        "--predocked",
        action="store_true",
        help="place rescuer at settled adjacent-face dock pose before running the roll",
    )
    args = parser.parse_args()
    if args.predocked and not args.cube_obj:
        parser.error("--predocked requires a cube")
    if args.fixed_grip:
        if not args.cube_obj or args.cube_control or args.patient_raise_deg:
            parser.error(
                "fixed grip requires a cube, a present patient and no patient lift stroke"
            )
        args.presentation = "carry-pose"
    if not 0 <= args.patient_raise_deg <= 45:
        parser.error("patient lift stroke must be in [0,45] degrees")
    if args.cube_control and args.cube_obj is None:
        parser.error("--cube-control requires --cube-obj")
    if not (
        np.isfinite(args.cube_mass_g)
        and args.cube_mass_g > 0
        and np.isfinite(args.cube_friction)
        and args.cube_friction > 0
        and np.isfinite(args.corner_friction) and args.corner_friction > 0
    ):
        parser.error("cube mass and friction must be finite and positive")
    if not (
        0 <= args.power <= 1
        and 0 <= args.delay_ms <= 4500
        and np.isfinite(args.offset_mm)
    ):
        parser.error("require finite offset, power in [0,1] and delay in [0,4500] ms")
    boxes = collision_boxes(args.cube_obj) if args.cube_obj else None
    corner_friction = None if args.uniform_cube else args.corner_friction
    xml = scene_xml(boxes, args.cube_mass_g, args.cube_friction, args.fixed_grip, corner_friction)
    if args.patient_fork_stl:
        from printed_fork import add_printed_fork
        root = ET.fromstring(xml)
        fork = root.find(".//body[@name='patient_lift_fork']")
        add_printed_fork(root, fork, args.patient_fork_stl, "patient_", replay.nominal_values())
        xml = ET.tostring(root, encoding="unicode")
    model = mj.MjModel.from_xml_string(xml)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "scene.xml").write_text(xml)
    summary = dict(
        mujoco=mj.__version__,
        scene_sha256=hashlib.sha256(xml.encode()).hexdigest(),
        offset_mm=args.offset_mm,
        power=args.power,
        delay_ms=args.delay_ms,
        sides={},
    )
    if args.patient_fork_stl:
        summary["patient_fork_stl_sha256"] = hashlib.sha256(args.patient_fork_stl.read_bytes()).hexdigest()
    if boxes:
        summary.update(
            cube_control=args.cube_control,
            fixed_grip=args.fixed_grip,
            predocked=args.predocked,
            presentation=args.presentation,
            patient_raise_deg=args.patient_raise_deg,
            cube_mass_g=args.cube_mass_g,
            cube_friction=args.cube_friction,
            corner_friction=corner_friction,
            sdk_cube_sha256=hashlib.sha256(args.cube_obj.read_bytes()).hexdigest(),
            cube_boxes=boxes,
        )
    for side, name in ((1, "right"), (-1, "left")):
        frames, summary["sides"][name] = run(model, side, args)
        (args.output / (name + ".json")).write_text(json.dumps(frames))
    (args.output / "run.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

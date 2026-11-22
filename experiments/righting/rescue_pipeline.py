"""Continuous three-stage free-body righting pipeline and no-cube control.

Stage A carries a free light cube welded to the rescuer wrist from standoff,
lowers to drop height, deactivates the weld on release, and backs off.
Stage B raises the sideways patient's lift into the dropped cube.
Stage C drives continuously around the patient without state resets or applied
cube forces to a belly/fork-side lever approach and executes the lift-assisted
righting stroke, followed by withdrawal. `--no-cube` runs the matching control
arm along the same continuous approach without a cube.

`--body DIR` gives both robots the real outer shape (simulacrum/body_mesh.py build output); without it they
are the parametric boxes. `--override name=value` (repeatable, JSON values) changes table entries for both
robots, e.g. the geometry corrections in evidence/body-shape. Neither changes the default run.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from simulacrum import mjcf, replay
from light_cube import add_cube, collision_boxes
from run import axis_rotation, patient_copy, pose


FRAME_DT = 1.0 / 30.0


def scene_values(overrides=None):
    """The table values both robots use: the replay's nominal set on an open table, then the overrides."""
    values = dict(replay.nominal_values(), **replay.OPEN_TABLE)
    unknown = sorted(set(overrides or {}) - set(values))
    assert not unknown, f"unknown parameters: {unknown}"
    values.update(overrides or {})
    return values


def scene_xml(boxes, mass_g, friction, corner_friction,
              exclude_robot_contacts, with_cube=True, values=None, body=None):
    """Build two-robot MuJoCo XML with optional free cube and exclusion."""
    values = scene_values() if values is None else values
    root = ET.fromstring(
        mjcf.build_mjcf(values) if body is None else mjcf.build_mjcf(values, body=body)
    )
    world = root.find("worldbody")
    rescuer = world.find("body[@name='chassis']")
    patient = patient_copy(rescuer)
    world.append(patient)
    for geom in world.findall("geom"):
        geom.set("conaffinity", "7" if with_cube else "3")
    for body, group, other in ((rescuer, "1", "2"), (patient, "2", "1")):
        for geom in body.iter("geom"):
            if geom.get("contype", "1") != "0":
                geom.set("contype", group)
                geom.set("conaffinity", other)
    for section in ("equality", "actuator"):
        parent = root.find(section)
        parent.extend([patient_copy(entry) for entry in parent])
    if with_cube:
        add_cube(world, boxes, mass_g, friction, corner_friction)
        equality = root.find("equality")
        if equality is None:
            equality = ET.SubElement(root, "equality")
        ET.SubElement(
            equality, "weld", name="carry_weld", body1="lift_fork",
            body2="cube", relpose="0.026 0 -0.0125 1 0 0 0", active="false",
        )
    if exclude_robot_contacts:
        contact = root.find("contact")
        if contact is None:
            contact = ET.SubElement(root, "contact")
        r_names = {b.get("name") for b in rescuer.iter("body") if b.get("name")}
        p_names = {b.get("name") for b in patient.iter("body") if b.get("name")}
        for a_name in sorted(r_names):
            for b_name in sorted(p_names):
                ET.SubElement(contact, "exclude", body1=a_name, body2=b_name)
    return ET.tostring(root, encoding="unicode")


def sideways_quat(side):
    """Quaternion placing patient on right (side=1) or left (side=-1) side."""
    out = np.zeros(4)
    mj.mju_mulQuat(
        out,
        axis_rotation([0, 0, 1], side * np.pi / 2),
        axis_rotation([1, 0, 0], side * np.pi / 2),
    )
    return out


def make_geom_sets(model):
    """Precompute geom index sets for zero-allocation step contact probes."""
    names = [model.geom(i).name for i in range(model.ngeom)]
    cube_g = {i for i, n in enumerate(names) if n.startswith("cube_")}
    # The fork's contact geoms: the parametric crossbar and side plates, or the real fork's pieces.
    pf_g = {
        i for i, n in enumerate(names)
        if n.startswith(("patient_lift_fork_geom", "patient_fork_side_", "patient_fork_hull_"))
    }
    rf_g = {
        i for i, n in enumerate(names)
        if n.startswith(("lift_fork_geom", "fork_side_", "fork_hull_"))
    }
    pat_g = {i for i, n in enumerate(names) if n.startswith("patient_")}
    env_g = {i for i, n in enumerate(names) if n in ("floor", "table")}
    act_g = set(range(model.ngeom)) - cube_g - pat_g - env_g
    return dict(cube=cube_g, pf=pf_g, rf=rf_g, pat=pat_g, act=act_g)


def step_and_sample(model, data, gsets, frames, next_frame_t, has_cube):
    """Advance one physics step, inspect contacts, and record 30 Hz frames."""
    mj.mj_step(model, data)
    pc = rc = dc = False
    point = []
    for contact in data.contact:
        ga, gb = contact.geom
        if ga in gsets["cube"] or gb in gsets["cube"]:
            other = gb if ga in gsets["cube"] else ga
            if other in gsets["pf"]:
                pc = True
            if other in gsets["rf"]:
                rc = True
                if not point:
                    point = [contact.pos.tolist()]
        if ((ga in gsets["act"] and gb in gsets["pat"])
                or (gb in gsets["act"] and ga in gsets["pat"])):
            dc = True
            if ga in gsets["rf"] or gb in gsets["rf"]:
                if not has_cube:
                    rc = True
                if not point:
                    point = [contact.pos.tolist()]
    if data.time + 1e-9 >= next_frame_t:
        frame = dict(
            t=float(data.time),
            contact=point,
            poses=[pose(data, ""), pose(data, "patient_")],
        )
        if has_cube:
            cube = data.body("cube")
            frame["cube"] = [*cube.xpos, *cube.xquat]
        frames.append(frame)
        next_frame_t += FRAME_DT
    return pc, rc, dc, next_frame_t


def patient_tilt_deg(data):
    """Angle in degrees between patient chassis +Z axis and world +Z."""
    up_z = float(np.clip(data.body("patient_chassis").xmat[8], -1.0, 1.0))
    return float(math.degrees(math.acos(up_z)))


def init_state(model, data, side, args):
    """Initialize sideways patient and rescuer (carrying cube unless no_cube)."""
    data.joint("patient_root").qpos[:3] = (0.0, 0.0, 0.039)
    data.joint("patient_root").qpos[3:] = sideways_quat(side)
    lo, hi = model.actuator("lift").ctrlrange
    for prefix in ("", "patient_"):
        is_rescuer = prefix == ""
        lift_0 = (lo if args.no_cube else hi) if is_rescuer else lo
        data.joint(prefix + "lift_joint").qpos[0] = lift_0
        data.joint(prefix + "wrist_joint").qpos[0] = -lift_0
        if mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, prefix + "lift_lower_joint") >= 0:
            data.joint(prefix + "lift_lower_joint").qpos[0] = lift_0  # the real body's lower links
        data.actuator(prefix + "lift").ctrl[0] = lift_0
    mj.mj_forward(model, data)
    fork_center = data.geom("patient_lift_fork_geom").xpos.copy()
    half_depth = model.geom("patient_lift_fork_geom").size[0]
    lane_y = fork_center[1] + side * (0.022 + half_depth + 0.0005)
    target_y = lane_y + side * args.offset_mm * 0.001
    standoff = args.approach_standoff_mm * 0.001
    data.joint("root").qpos[:3] = (fork_center[0] - 0.056 - standoff, target_y, 0.001)
    args._cube_drop_target = (fork_center[0], target_y, 0.022)
    mj.mj_forward(model, data)
    if not args.no_cube:
        wrist = data.body("lift_fork")
        cpos = data.joint("cube_root").qpos
        cpos[:3] = wrist.xpos + wrist.xmat.reshape(3, 3) @ np.array([0.026, 0.0, -0.0125])
        cpos[3:] = wrist.xquat
        mj.mj_forward(model, data)
        data.eq_active[model.equality("carry_weld").id] = 1
        mj.mj_forward(model, data)


def tread_drive(values, measured_rad_s, speed_mmps):
    """Static-friction feedforward + proportional speed trim duty in [-1, 1]."""
    measured = measured_rad_s * values["wheel_front_radius_mm"]
    friction = values["tread_friction_power_fraction"]
    tread = values["tread_noload_speed_mmps"]
    raw = (
        np.sign(speed_mmps) * friction
        + (speed_mmps + 0.6 * (speed_mmps - measured)) / tread
    )
    return float(np.clip(raw, -1.0, 1.0))


def record_stage_a_metrics(data, args, start_t, released_t, phase, log):
    """Score cube drop pose against _cube_drop_target and log Stage A metrics."""
    assert data is not None and isinstance(log, dict), "valid data and log required"
    cube = data.body("cube")
    drop = np.array(args._cube_drop_target)
    xmat = cube.xmat.reshape(3, 3)
    on_face = min(float(math.degrees(math.acos(np.clip(abs(xmat[2, k]), -1.0, 1.0)))) for k in range(3))
    log.update(
        stage_a_release_time_s=None if released_t is None else float(released_t),
        stage_a_rescuer_end_x_mm=float(data.body("chassis").xpos[0] * 1000.0),
        stage_a_cube_end_xyz_mm=[float(v * 1000.0) for v in cube.xpos],
        stage_a_cube_drop_target_xyz_mm=[float(v * 1000.0) for v in drop],
        stage_a_cube_placement_err_mm=float(np.linalg.norm(cube.xpos - drop) * 1000.0),
        stage_a_cube_placement_err_xy_mm=float(np.linalg.norm(cube.xpos[:2] - drop[:2]) * 1000.0),
        stage_a_cube_placement_err_z_mm=float(abs(cube.xpos[2] - drop[2]) * 1000.0),
        stage_a_cube_face_tilt_min_deg=on_face,
        stage_a_cube_face_upright=bool(on_face < 15.0),
        stage_a_phase_end=phase,
        stage_a_elapsed_s=float(data.time - start_t),
    )


def stage_a_place(model, data, args, gsets, frames, next_frame_t, log):
    """Carry cube to drop point, release weld, and reverse to standoff."""
    assert model is not None and data is not None, "valid MuJoCo model/data required"
    values = args._values
    lo, hi = model.actuator("lift").ctrlrange
    weld_id = model.equality("carry_weld").id
    stop_x = args._cube_drop_target[0] - 0.048
    reverse_x = stop_x - args.recovery_dock_standoff_mm * 0.001
    release_angle = math.radians(args.release_lift_deg)
    start_t = data.time
    released_t = None
    phase = "approach"
    for _ in range(round(6.0 / model.opt.timestep)):
        cx = data.body("chassis").xpos[0]
        if phase == "approach" and cx >= stop_x:
            phase = "lower"
        if phase == "lower" and data.joint("lift_joint").qpos[0] <= release_angle:
            if released_t is None:
                data.eq_active[weld_id] = 0
                released_t = data.time
            phase = "reverse"
        if phase == "reverse" and cx <= reverse_x:
            break
        spd = args.carry_speed_mmps if phase == "approach" else (0.0 if phase == "lower" else -args.carry_speed_mmps)
        drv = tread_drive(values, data.joint("wheel_fl_joint").qvel[0], spd)
        data.actuator("left_wheel").ctrl[0] = drv
        data.actuator("right_wheel").ctrl[0] = drv
        if phase in ("approach", "lower"):
            data.actuator("lift").ctrl[0] = hi if phase == "approach" else math.radians(args.drop_height_deg)
        data.actuator("patient_lift").ctrl[0] = lo
        _, _, _, next_frame_t = step_and_sample(model, data, gsets, frames, next_frame_t, True)
    record_stage_a_metrics(data, args, start_t, released_t, phase, log)
    return next_frame_t


def stage_b_engage(model, data, args, gsets, frames, next_frame_t, log):
    """Raise patient lift toward dropped cube while rescuer waits at standoff."""
    lo, _ = model.actuator("lift").ctrlrange
    max_steps = round(args.engage_duration_s / model.opt.timestep)
    start_t = data.time
    fork_steps = 0
    peak_tilt = 0.0
    cube_q0 = data.body("cube").xquat.copy()
    stroke = math.radians(args.patient_raise_deg)
    for _ in range(max_steps):
        ramp = float(np.clip((data.time - start_t) / 0.5, 0.0, 1.0))
        data.actuator("patient_lift").ctrl[0] = lo + stroke * ramp
        data.actuator("left_wheel").ctrl[0] = 0.0
        data.actuator("right_wheel").ctrl[0] = 0.0
        pc, _, _, next_frame_t = step_and_sample(model, data, gsets, frames, next_frame_t, True)
        fork_steps += int(pc)
        peak_tilt = max(peak_tilt, patient_tilt_deg(data))
    cube_q1 = data.body("cube").xquat.copy()
    net_rot = float(math.degrees(2.0 * math.acos(np.clip(abs(float(np.dot(cube_q0, cube_q1))), 0.0, 1.0))))
    fork_s = fork_steps * model.opt.timestep
    log.update(
        stage_b_patient_fork_on_cube_s=float(fork_s),
        stage_b_held_through_handoff=bool(fork_s > 0.4),
        stage_b_peak_patient_tilt_deg=float(peak_tilt),
        stage_b_cube_rotation_during_engage_deg=net_rot,
        stage_b_elapsed_s=float(data.time - start_t),
    )
    return next_frame_t


def drive_to_pose(model, data, target_xy, target_yaw, lift_ctrl,
                  pat_lift_ctrl, gsets, frames, next_frame_t, has_cube):
    """Turn and drive continuously to (target_xy, target_yaw) on flat table."""
    steps = round(6.0 / model.opt.timestep)
    for phase in ("to_wp", "align"):
        for _ in range(steps):
            pos = data.body("chassis").xpos[:2]
            xmat = data.body("chassis").xmat.reshape(3, 3)
            yaw = math.atan2(xmat[1, 0], xmat[0, 0])
            if phase == "to_wp":
                dxy = target_xy - pos
                dist = float(np.hypot(dxy[0], dxy[1]))
                if dist <= 0.003:
                    break
                des = math.atan2(dxy[1], dxy[0])
                err = math.atan2(math.sin(des - yaw), math.cos(des - yaw))
                turn_only = abs(err) > math.radians(10.0)
                v_cmd = 0.0 if turn_only else float(np.clip(3.5 * dist + 0.22, 0.22, 0.55))
                w_cmd = float(
                    np.clip(2.2 * err + 0.18 * np.sign(err), -0.65, 0.65)
                    if turn_only else np.clip(1.8 * err, -0.35, 0.35)
                )
            else:
                err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
                if abs(err) <= math.radians(1.5):
                    break
                v_cmd = 0.0
                w_cmd = float(np.clip(2.0 * err + 0.18 * np.sign(err), -0.6, 0.6))
            data.actuator("left_wheel").ctrl[0] = float(np.clip(v_cmd - w_cmd, -1.0, 1.0))
            data.actuator("right_wheel").ctrl[0] = float(np.clip(v_cmd + w_cmd, -1.0, 1.0))
            data.actuator("lift").ctrl[0] = lift_ctrl
            data.actuator("patient_lift").ctrl[0] = pat_lift_ctrl
            _, _, _, next_frame_t = step_and_sample(model, data, gsets, frames, next_frame_t, has_cube)
    return next_frame_t


def is_rescuer_clear(data, gsets):
    """Return True if no rescuer geom touches patient or cube geoms."""
    assert data is not None and "act" in gsets, "valid data and geom sets required"
    targets = gsets["pat"] | gsets["cube"]
    return not any(
        ((ga in gsets["act"] and gb in targets) or (gb in gsets["act"] and ga in targets))
        for ga, gb in (c.geom for c in data.contact)
    )


def execute_lever_stroke(model, data, pat_lift_ctrl, gsets, frames,
                         next_frame_t, args):
    """Execute contact-triggered belly/fork-side lift-and-push stroke."""
    assert model is not None and data is not None, "valid MuJoCo model/data required"
    lo, hi = model.actuator("lift").ctrlrange
    t0, hit = data.time, None
    best = worst = patient_tilt_deg(data)
    direct_steps = dual_steps = rc_steps = 0
    for _ in range(round(args.recovery_duration_s / model.opt.timestep)):
        t = data.time - t0
        elapsed = (t - hit) if hit is not None else (max(-1.0, t - 1.1) if t > 1.1 else -1.0)
        rise = float(np.clip((elapsed - 0.09) / 0.23, 0.0, 1.0))
        lower = float(np.clip((elapsed - 0.09 - 0.75 - 1.0) / 0.45, 0.0, 1.0))
        data.actuator("lift").ctrl[0] = lo + (hi - lo) * (0.23 + 0.77 * rise) * (1.0 - lower)
        drv = 0.92 if elapsed < 0.84 else (-0.50 if elapsed < 1.64 else 0.0)
        data.actuator("left_wheel").ctrl[0] = drv
        data.actuator("right_wheel").ctrl[0] = drv
        data.actuator("patient_lift").ctrl[0] = pat_lift_ctrl
        pc, rc, dc, next_frame_t = step_and_sample(model, data, gsets, frames, next_frame_t, not args.no_cube)
        if hit is None and (rc or dc):
            hit = t
        direct_steps += int(dc)
        dual_steps += int(pc and rc)
        rc_steps += int(rc)
        tilt = patient_tilt_deg(data)
        best, worst = min(best, tilt), max(worst, tilt)
    return next_frame_t, dict(
        first_contact_s=None if hit is None else float(hit),
        direct_robot_contact_steps=direct_steps,
        both_forks_on_cube_steps=dual_steps,
        rescuer_fork_cube_steps=rc_steps,
        final_patient_tilt_deg=round(patient_tilt_deg(data), 3),
        closest_to_upright_deg=round(best, 3),
        farthest_from_upright_deg=round(worst, 3),
        rescuer_clear_at_end=is_rescuer_clear(data, gsets),
    )


def stage_c_recovery(model, data, side, pat_lift_ctrl, args, gsets,
                     frames, next_frame_t, log):
    """Navigate continuously to belly/cube lever pose and run righting stroke."""
    lo, hi = model.actuator("lift").ctrlrange
    lift_h0 = lo + 0.23 * (hi - lo)
    ref_x = 0.0113 if args.no_cube else float(data.body("cube").xpos[0])
    yaw = math.radians(168.7) * side
    fwd = np.array([math.cos(yaw), math.sin(yaw)])
    target_pt = np.array([ref_x + 0.025, (0.038 + args.offset_mm * 0.001) * side])
    approach_xy = target_pt - 0.140 * fwd
    wp1 = np.array([float(data.body("chassis").xpos[0]), -0.15 * side])
    wp2 = np.array([float(approach_xy[0]) + 0.02, -0.15 * side])
    has_cube = not args.no_cube
    t0 = data.time
    for wp, wyaw in ((wp1, 0.0), (wp2, 0.0), (approach_xy, yaw)):
        next_frame_t = drive_to_pose(
            model, data, wp, wyaw, lift_h0, pat_lift_ctrl,
            gsets, frames, next_frame_t, has_cube,
        )
    for _ in range(round(0.15 / model.opt.timestep)):
        data.actuator("left_wheel").ctrl[0] = 0.0
        data.actuator("right_wheel").ctrl[0] = 0.0
        _, _, _, next_frame_t = step_and_sample(model, data, gsets, frames, next_frame_t, has_cube)
    start_tilt = patient_tilt_deg(data)
    next_frame_t, stats = execute_lever_stroke(
        model, data, pat_lift_ctrl, gsets, frames, next_frame_t, args
    )
    prefix = "nocube_" if args.no_cube else "stage_c_"
    log[prefix + "start_patient_tilt_deg"] = round(start_tilt, 3)
    for k, val in stats.items():
        log[prefix + k] = val
    log[prefix + "elapsed_s"] = float(data.time - t0)
    return next_frame_t


def pad_paired_frames(left_right_pairs):
    """Equalize frame list lengths by extending the settled end pose."""
    assert all(len(seq) > 0 for seq in left_right_pairs), "nonempty frame sequences required"
    max_len = max(len(seq) for seq in left_right_pairs)
    for seq in left_right_pairs:
        for _ in range(max_len - len(seq)):
            tail = dict(seq[-1])
            tail["t"] = round(tail["t"] + FRAME_DT, 6)
            tail["contact"] = []
            seq.append(tail)


def run_one(model, side, args):
    """Run one complete side trial cleanly without artificial impulses."""
    assert model is not None and side in (-1, 1), "valid model and side in (-1, 1) required"
    data = mj.MjData(model)
    gsets = make_geom_sets(model)
    frames = []
    next_t = 0.0
    log = dict(side=side)
    init_state(model, data, side, args)
    if args.no_cube:
        pat_lift = model.actuator("lift").ctrlrange[0]
    else:
        next_t = stage_a_place(model, data, args, gsets, frames, next_t, log)
        next_t = stage_b_engage(model, data, args, gsets, frames, next_t, log)
        pat_lift = float(data.actuator("patient_lift").ctrl[0])
    stage_c_recovery(model, data, side, pat_lift, args, gsets, frames, next_t, log)
    return frames, log


def paired_check(path_contact, path_nocontact):
    """Verify contact-allowed vs contact-excluded runs are non-identical."""
    with open(path_contact, encoding="utf-8") as f:
        run_a = json.load(f)
    with open(path_nocontact, encoding="utf-8") as f:
        run_b = json.load(f)
    diffs = []
    for side_key in ("right", "left"):
        sa = run_a.get("sides", {}).get(side_key, {})
        sb = run_b.get("sides", {}).get(side_key, {})
        for k in sorted(set(sa) | set(sb)):
            va, vb = sa.get(k), sb.get(k)
            if isinstance(va, float) and isinstance(vb, float):
                if abs(va - vb) > 1e-5:
                    diffs.append([side_key, k, va, vb])
            elif va != vb:
                diffs.append([side_key, k, va, vb])
    return {"ok": len(diffs) > 0, "differing_fields": diffs}


def parse_args(argv=None):
    """Parse and validate CLI arguments for rescue_pipeline."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--cube-obj", type=Path)
    parser.add_argument("--no-cube", action="store_true")
    parser.add_argument("--offset-mm", type=float, default=0.0)
    parser.add_argument("--cube-mass-g", type=float, default=50.0)
    parser.add_argument("--cube-friction", type=float, default=0.5)
    parser.add_argument("--corner-friction", type=float, default=0.9)
    parser.add_argument("--approach-standoff-mm", type=float, default=120.0)
    parser.add_argument("--recovery-dock-standoff-mm", type=float, default=80.0)
    parser.add_argument("--carry-speed-mmps", type=float, default=40.0)
    parser.add_argument("--release-lift-deg", type=float, default=15.0)
    parser.add_argument("--drop-height-deg", type=float, default=0.0)
    parser.add_argument("--patient-raise-deg", type=float, default=15.0)
    parser.add_argument("--engage-duration-s", type=float, default=1.5)
    parser.add_argument("--recovery-duration-s", type=float, default=3.8)
    parser.add_argument("--exclude-robot-contacts", action="store_true")
    parser.add_argument("--body", type=Path, help="built body directory (simulacrum/body_mesh.py)")
    parser.add_argument("--override", action="append", default=[], help="name=value, value as JSON")
    args = parser.parse_args(argv)
    try:
        args._overrides = {k: json.loads(x) for k, x in (o.split("=", 1) for o in args.override)}
        args._values = scene_values(args._overrides)
    except (ValueError, AssertionError) as error:
        parser.error(f"--override: {error}")
    if not args.no_cube and args.cube_obj is None:
        parser.error("--cube-obj required unless --no-cube")
    if not (args.cube_mass_g > 0 and args.cube_friction > 0 and args.corner_friction > 0):
        parser.error("cube mass and friction values must be positive")
    if not np.isfinite(args.offset_mm):
        parser.error("--offset-mm must be finite")
    return args


def main():
    if len(sys.argv) >= 4 and sys.argv[1] == "--self-check":
        result = paired_check(sys.argv[2], sys.argv[3])
        print(json.dumps(result, indent=2))
        sys.exit(0 if result["ok"] else 2)
    args = parse_args()
    overrides = args._overrides
    body = None
    if args.body is not None:
        from simulacrum import body_mesh
        body = body_mesh.load(args.body)
    boxes = None if args.no_cube else collision_boxes(args.cube_obj)
    if boxes is not None:
        # collision_boxes normalises the SDK shell to the engine's 44 mm; scale it to the cube as built.
        scale = args._values["cube_outer_mm"] / 44.0
        boxes = [([c * scale for c in pos], [h * scale for h in half]) for pos, half in boxes]
    xml = scene_xml(
        boxes, args.cube_mass_g, args.cube_friction, args.corner_friction,
        args.exclude_robot_contacts, with_cube=not args.no_cube, values=args._values, body=body,
    )
    model = mj.MjModel.from_xml_string(xml)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "scene.xml").write_text(xml, encoding="utf-8")
    summary = dict(
        mujoco=mj.__version__,
        scene_sha256=hashlib.sha256(xml.encode()).hexdigest(),
        exclude_robot_contacts=args.exclude_robot_contacts,
        offset_mm=args.offset_mm,
        no_cube=args.no_cube,
        cube_friction=args.cube_friction,
        fixed_grip=False,
        body=None if body is None else str(args.body),
        overrides=overrides,
        cube_boxes=boxes or [],
        banner=(
            "Continuous free-body righting chain (no-cube control)"
            if args.no_cube
            else "Continuous free-body cube rescue chain"
        ),
        sides={},
    )
    paired_frames = []
    for side, name in ((1, "right"), (-1, "left")):
        frames, summary["sides"][name] = run_one(model, side, args)
        paired_frames.append((name, frames))
    pad_paired_frames([f for _, f in paired_frames])
    for name, frames in paired_frames:
        (args.output / (name + ".json")).write_text(json.dumps(frames), encoding="utf-8")
    (args.output / "run.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

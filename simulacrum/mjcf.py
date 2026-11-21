"""MJCF generator for the Vector simulacrum. Authored-by: CPCS.

Every number in the generated model comes from the parameter table (through
the value map from params.values or params.sample) or is derived from those
numbers here. The table is in millimetres and grams; MJCF is in metres,
kilograms and radians. No MuJoCo import: the output is a string.

Robot frame: origin on the ground under the centre of the front axle,
x forward, y left, z up. The table top is the plane z = 0.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Sequence, Tuple

MM = 0.001
GRAM = 0.001
MAX_OBSTACLES = 16
CLIFF_NAMES = ("front_left", "front_right", "back_left", "back_right")
# The robot's own geoms are in this group and the scene's in group 0, so that the time-of-flight
# sensor's rays (robot.py) can be cast at the scene alone. Contact does not use groups.
ROBOT_GEOM_GROUP = 1
# Where the model may carry a light cube: nowhere (the default) or in the lift.
CUBE_MOUNTS = (None, "lift")


def num(value: float) -> str:
    """Format one number the same way every time."""
    assert math.isfinite(value), "non-finite number in model"
    text = f"{value:.9g}"
    return "0" if text in ("-0", "0") else text


def vec(values: Sequence[float]) -> str:
    assert len(values) > 0, "empty vector"
    return " ".join(num(float(v)) for v in values)


def lift_angle_of_height(v: Dict[str, Any], height_mm: float) -> float:
    """Shoulder angle (rad, up positive) for a lift height, as the firmware converts it."""
    clipped = min(max(height_mm, v["lift_height_min_mm"]), v["lift_height_max_mm"])
    ratio = (clipped - v["lift_base_position_mm"][2]) / v["lift_arm_length_mm"]
    assert -1.0 <= ratio <= 1.0, "lift height unreachable with this arm length"
    return math.asin(ratio)


def lift_height_of_angle(v: Dict[str, Any], angle_rad: float) -> float:
    assert math.isfinite(angle_rad), "lift angle must be finite"
    return math.sin(angle_rad) * v["lift_arm_length_mm"] + v["lift_base_position_mm"][2]


def _part_masses(v: Dict[str, Any]) -> Dict[str, float]:
    """Masses in grams. The chassis is what is left of the total."""
    arm = v["lift_mass_g"] * v["lift_arm_mass_fraction"]
    fork = v["lift_mass_g"] - arm
    chassis = v["total_mass_g"] - v["head_mass_g"] - v["lift_mass_g"] - 4.0 * v["wheel_mass_g"]
    assert chassis > 0.0, "part masses exceed the total mass"
    assert arm > 0.0 and fork > 0.0, "lift mass fraction must be inside 0..1"
    return {"chassis": chassis, "head": v["head_mass_g"], "arm": arm, "fork": fork, "wheel": v["wheel_mass_g"]}


def _fork_center_wrt_wrist(v: Dict[str, Any]) -> Tuple[float, float]:
    """Centre (x, z) of the lift crossbar relative to the wrist, in mm."""
    x_c = 0.5 * (v["lift_front_wrt_wrist_mm"] + v["lift_back_wrt_wrist_mm"])
    z_c = v["lift_xbar_top_wrt_wrist_mm"] - 0.5 * v["lift_xbar_height_mm"]
    assert v["lift_front_wrt_wrist_mm"] > v["lift_back_wrt_wrist_mm"], "lift plate has no depth"
    assert v["lift_xbar_height_mm"] > 0.0, "lift crossbar has no height"
    return x_c, z_c


def _moving_part_points(v: Dict[str, Any]) -> List[Tuple[float, float, float]]:
    """(mass g, x mm, z mm) of every part except the chassis, lift down, head level."""
    m = _part_masses(v)
    a0 = lift_angle_of_height(v, v["lift_height_min_mm"])
    sx, _, sz = v["lift_base_position_mm"]
    arm_len = v["lift_arm_length_mm"]
    wrist = (sx + arm_len * math.cos(a0), sz + arm_len * math.sin(a0))
    fx, fz = _fork_center_wrt_wrist(v)
    points = [
        (m["head"], v["neck_joint_position_mm"][0], v["neck_joint_position_mm"][2]),
        (m["arm"], 0.5 * (sx + wrist[0]), 0.5 * (sz + wrist[1])),
        (m["fork"], wrist[0] + fx, wrist[1] + fz),
        (2.0 * m["wheel"], wheel_x_mm(v)[0], v["wheel_front_radius_mm"]),
        (2.0 * m["wheel"], wheel_x_mm(v)[1], v["wheel_rear_radius_mm"]),
    ]
    assert all(p[0] > 0.0 for p in points), "non-positive part mass"
    assert len(points) == 5, "part list changed"
    return points


def derive(v: Dict[str, Any]) -> Dict[str, Any]:
    """Quantities computed from the table: chassis inertial, limits, gear ratio."""
    masses = _part_masses(v)
    points = _moving_part_points(v)
    total = v["total_mass_g"]
    com_x = (total * v["com_x_mm"] - sum(p[0] * p[1] for p in points)) / masses["chassis"]
    com_z = (total * v["com_z_mm"] - sum(p[0] * p[2] for p in points)) / masses["chassis"]
    length, width = v["chassis_length_mm"], v["chassis_width_mm"]
    height = v["chassis_top_z_mm"] - v["chassis_ground_clearance_mm"]
    assert height > 0.0, "chassis has no height"
    m_kg = masses["chassis"] * GRAM
    inertia = [m_kg / 12.0 * ((b * MM) ** 2 + (c * MM) ** 2)
               for b, c in ((width, height), (length, height), (length, width))]
    ticks = v["control_period_ms"] / v["sim_timestep_ms"]
    assert abs(ticks - round(ticks)) < 1e-9 and ticks >= 1.0, "sim_timestep_ms must divide control_period_ms"
    return {
        "masses_g": masses, "chassis_com_mm": (com_x, 0.0, com_z), "chassis_inertia": inertia,
        "chassis_height_mm": height,
        "lift_angle_min": lift_angle_of_height(v, v["lift_height_min_mm"]),
        "lift_angle_max": lift_angle_of_height(v, v["lift_height_max_mm"]),
        "rear_gear": v["wheel_front_radius_mm"] / v["wheel_rear_radius_mm"],
        "substeps": int(round(ticks)),
    }


def _contact_attrs(v: Dict[str, Any], friction: float) -> str:
    """Attributes of a robot geom that touches the world. Priority 1 makes its friction the one used."""
    assert friction > 0.0, "friction must be positive"
    tc = v["contact_time_constant_ms"] * 0.001
    assert tc >= 2.0 * v["sim_timestep_ms"] * 0.001, "contact time constant below two timesteps"
    return (f'contype="1" conaffinity="0" priority="1" condim="3" friction="{num(friction)} 0.005 0.0001" '
            f'solref="{num(tc)} 1"')


def tread_drive(v: Dict[str, Any]) -> Dict[str, float]:
    """The tread motor as a linear motor with constant friction, in SI units at the front wheel.

    torque = stall * power - damping * wheel speed, opposed by a constant friction torque.
    Held at a steady speed this needs power = speed / no-load speed + friction / stall,
    which is the shape of the firmware's feedforward line; the no-load speed and the
    friction fraction are read off that line. The stall torque is assumed.
    """
    stall = v["wheel_stall_torque_mNm"] * 0.001
    noload_radps = v["tread_noload_speed_mmps"] / v["wheel_front_radius_mm"]
    assert stall > 0.0 and noload_radps > 0.0, "stall torque and no-load speed must be positive"
    assert 0.0 <= v["tread_friction_power_fraction"] < 1.0, "friction fraction must be inside 0..1"
    return {"stall_Nm": stall, "damping_Nms_per_rad": stall / noload_radps,
            "friction_Nm": v["tread_friction_power_fraction"] * stall}


def _wheel_xml(v: Dict[str, Any], name: str, x_mm: float, y_mm: float, radius_mm: float, driven: bool) -> str:
    assert radius_mm > 0.0 and v["tread_width_mm"] > 0.0, "wheel must have size"
    half_w = 0.5 * v["tread_width_mm"] * MM
    mass = v["wheel_mass_g"] * GRAM
    arm = v["wheel_armature_gcm2"] * 1e-7
    loss = f' frictionloss="{num(tread_drive(v)["friction_Nm"])}"' if driven else ""
    return (
        f'    <body name="{name}" pos="{vec((x_mm * MM, y_mm * MM, radius_mm * MM))}">\n'
        f'      <joint name="{name}_joint" type="hinge" axis="0 1 0" armature="{num(arm)}"{loss}/>\n'
        f'      <geom name="{name}_geom" type="cylinder" fromto="0 {num(-half_w)} 0 0 {num(half_w)} 0" '
        f'size="{num(radius_mm * MM)}" mass="{num(mass)}" rgba="0.15 0.15 0.15 1" group="{ROBOT_GEOM_GROUP}" '
        f'{_contact_attrs(v, v["tread_friction"])}/>\n'
        f'    </body>\n'
    )


def wheel_x_mm(v: Dict[str, Any]) -> Tuple[float, float]:
    """x of the front and rear wheels in the model: the axles, each moved inward by tread_contact_inset_mm.
    The firmware's wheel base stays the axle-to-axle distance; the inset moves where the treads meet the table."""
    base, inset = v["wheel_base_mm"], v["tread_contact_inset_mm"]
    assert base > 0.0, "wheel base must be positive"
    assert 0.0 <= inset < 0.25 * base, "contact inset must be at least 0 and under a quarter of the wheel base"
    return -inset, -(base - inset)


def _tip_supports_xml(v: Dict[str, Any]) -> str:
    """Body-fixed cylinders round the real axles, a little smaller than the wheels, for tipping.

    With a contact inset the wheels touch the table inside the axles, which is right on a flat
    table but would move the edge the robot tips over. These sit tip_support_clearance_mm above
    the table at rest, so they carry nothing on the flat and take over once the body tilts: a
    robot balances with its centre of mass above the centre of the circle it rests on, whatever
    the circle's radius, so the tipping angles are the real axles' (wheelstand.py)."""
    inset, gap = v["tread_contact_inset_mm"], v["tip_support_clearance_mm"]
    assert gap > 0.0, "tip supports must clear the table"
    if inset <= 0.0:
        return ""
    half, half_w = 0.5 * v["tread_spacing_mm"] * MM, 0.5 * v["tread_width_mm"] * MM
    rf, rr, base = v["wheel_front_radius_mm"], v["wheel_rear_radius_mm"], v["wheel_base_mm"]
    assert gap < min(rf, rr), "tip supports would have no size"
    out = ""
    for name, x_mm, radius_mm in (("tip_front", 0.0, rf), ("tip_rear", -base, rr)):
        for side, y in (("l", half), ("r", -half)):
            out += (f'      <geom name="{name}_{side}" type="cylinder" fromto="{num(x_mm * MM)} {num(y - half_w)} '
                    f'{num(radius_mm * MM)} {num(x_mm * MM)} {num(y + half_w)} {num(radius_mm * MM)}" '
                    f'size="{num((radius_mm - gap) * MM)}" mass="0" rgba="0.15 0.15 0.15 0.3" '
                    f'group="{ROBOT_GEOM_GROUP}" {_contact_attrs(v, v["tread_friction"])}/>\n')
    return out


def _wheels_xml(v: Dict[str, Any]) -> str:
    half = 0.5 * v["tread_spacing_mm"]
    assert half > 0.5 * v["chassis_width_mm"], "treads overlap the chassis"
    rf, rr = v["wheel_front_radius_mm"], v["wheel_rear_radius_mm"]
    front, rear = wheel_x_mm(v)
    return (_wheel_xml(v, "wheel_fl", front, half, rf, True) + _wheel_xml(v, "wheel_fr", front, -half, rf, True)
            + _wheel_xml(v, "wheel_rl", rear, half, rr, False) + _wheel_xml(v, "wheel_rr", rear, -half, rr, False))


def _head_xml(v: Dict[str, Any]) -> str:
    lo, hi = math.radians(v["head_angle_min_deg"]), math.radians(v["head_angle_max_deg"])
    assert lo < 0.0 < hi, "head range must straddle level"
    bx, by, bz = (0.5 * s * MM for s in v["head_box_mm"])
    arm = v["servo_armature_gcm2"] * 1e-7
    cam, imu = v["head_camera_position_mm"], v["imu_position_mm"]
    res = v["camera_resolution_px"]
    assert len(cam) == 3 and len(imu) == 3 and len(res) == 2, "bad head vectors"
    # The IMU's frame is pitched against the head (positive is nose up, like the head joint, whose axis is -y).
    half = 0.5 * math.radians(v["imu_pitch_offset_deg"])
    imu_quat = (math.cos(half), 0.0, -math.sin(half), 0.0)
    assert abs(half) < 0.5, "IMU pitch offset is not a small angle"
    return (
        f'    <body name="head" pos="{vec([c * MM for c in v["neck_joint_position_mm"]])}">\n'
        f'      <joint name="head_joint" type="hinge" axis="0 -1 0" range="{num(lo)} {num(hi)}" '
        f'armature="{num(arm)}"/>\n'
        f'      <geom name="head_geom" type="box" size="{vec((bx, by, bz))}" '
        f'mass="{num(v["head_mass_g"] * GRAM)}" rgba="0.25 0.25 0.3 1" group="{ROBOT_GEOM_GROUP}" '
        f'{_contact_attrs(v, v["chassis_friction"])}/>\n'
        f'      <camera name="head_cam" pos="{vec([c * MM for c in cam])}" xyaxes="0 -1 0 0 0 1" '
        f'fovy="{num(v["camera_fov_vertical_deg"])}" resolution="{int(res[0])} {int(res[1])}"/>\n'
        f'      <site name="imu" pos="{vec([c * MM for c in imu])}" quat="{vec(imu_quat)}" size="0.001"/>\n'
        f'    </body>\n'
    )


def _carried_cube_xml(v: Dict[str, Any]) -> str:
    """A light cube held in the lift, as the engine places a carried cube (cube_carry_offset_mm from the
    wrist, on the level fork's axes), with a site at its accelerometer. The fork's body is the parent, so the
    two never collide with each other; the cube touches the world like the fork does."""
    half = 0.5 * v["cube_size_mm"]
    chip = v["cube_chip_position_mm"]
    yaw = math.radians(v["cube_chip_yaw_deg"])
    assert half > 0.0 and v["cube_mass_g"] > 0.0, "the cube must have size and mass"
    assert len(chip) == 3 and all(abs(c) < half for c in chip), "the chip must be inside the cube"
    assert len(v["cube_carry_offset_mm"]) == 3, "the carry offset has three axes"
    return (
        f'        <body name="cube" pos="{vec([c * MM for c in v["cube_carry_offset_mm"]])}">\n'
        f'          <geom name="cube_geom" type="box" size="{vec((half * MM,) * 3)}" '
        f'mass="{num(v["cube_mass_g"] * GRAM)}" rgba="0.85 0.85 0.8 1" group="{ROBOT_GEOM_GROUP}" '
        f'{_contact_attrs(v, v["chassis_friction"])}/>\n'
        f'          <site name="cube_chip" pos="{vec([c * MM for c in chip])}" '
        f'quat="{vec((math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)))}" size="0.001"/>\n'
        f'        </body>\n'
    )


def _lift_xml(v: Dict[str, Any], d: Dict[str, Any], cube: Any = None) -> str:
    assert cube in CUBE_MOUNTS, "a cube goes nowhere or in the lift"
    arm_len = v["lift_arm_length_mm"] * MM
    half_w = 0.5 * v["lift_xbar_width_mm"] * MM
    fx, fz = _fork_center_wrt_wrist(v)
    half_x = 0.5 * (v["lift_front_wrt_wrist_mm"] - v["lift_back_wrt_wrist_mm"]) * MM
    half_z = 0.5 * v["lift_xbar_height_mm"] * MM
    arm = v["servo_armature_gcm2"] * 1e-7
    assert d["lift_angle_min"] < d["lift_angle_max"], "lift range is empty"
    assert arm_len > 0.0 and half_w > 0.0, "lift must have size"
    return (
        f'    <body name="lift_arm" pos="{vec([c * MM for c in v["lift_base_position_mm"]])}">\n'
        f'      <joint name="lift_joint" type="hinge" axis="0 -1 0" '
        f'range="{num(d["lift_angle_min"])} {num(d["lift_angle_max"])}" armature="{num(arm)}"/>\n'
        f'      <geom name="lift_arm_geom" type="box" pos="{vec((0.5 * arm_len, 0, 0))}" '
        f'size="{vec((0.5 * arm_len, half_w, 0.002))}" mass="{num(d["masses_g"]["arm"] * GRAM)}" '
        f'contype="0" conaffinity="0" rgba="0.3 0.3 0.3 1" group="{ROBOT_GEOM_GROUP}"/>\n'
        f'      <body name="lift_fork" pos="{vec((arm_len, 0, 0))}">\n'
        f'        <joint name="wrist_joint" type="hinge" axis="0 -1 0"/>\n'
        f'        <geom name="lift_fork_geom" type="box" pos="{vec((fx * MM, 0, fz * MM))}" '
        f'size="{vec((half_x, half_w, half_z))}" mass="{num(d["masses_g"]["fork"] * GRAM)}" '
        f'rgba="0.3 0.3 0.3 1" group="{ROBOT_GEOM_GROUP}" {_contact_attrs(v, v["chassis_friction"])}/>\n'
        + (_carried_cube_xml(v) if cube == "lift" else "") +
        f'      </body>\n'
        f'    </body>\n'
    )


def _cliff_positions(v: Dict[str, Any]) -> List[Tuple[str, float, float]]:
    """(name, x mm, y mm) in the manual's index order: FL, FR, BL, BR."""
    assert tuple(v["cliff_sensor_order"]) == CLIFF_NAMES, "cliff sensor order differs from the manual"
    fx, rx, y = v["cliff_front_x_mm"], v["cliff_rear_x_mm"], v["cliff_y_mm"]
    assert fx > rx and y > 0.0, "cliff sensor layout is degenerate"
    return [("front_left", fx, y), ("front_right", fx, -y), ("back_left", rx, y), ("back_right", rx, -y)]


def _sites_xml(v: Dict[str, Any]) -> str:
    """Range sensor sites on the chassis. A cliff rangefinder looks along its site's +z axis; the
    time-of-flight site gives that sensor's position and axis, and robot.py casts its cone of rays."""
    out = ""
    h = v["cliff_sensor_height_mm"] * MM
    assert h > 0.0, "cliff sensors must sit above the ground"
    for name, x_mm, y_mm in _cliff_positions(v):
        out += (f'      <site name="cliff_{name}" pos="{vec((x_mm * MM, y_mm * MM, h))}" '
                f'zaxis="0 0 -1" size="0.001"/>\n')
    tilt = math.radians(v["tof_tilt_up_deg"])
    assert abs(tilt) < 0.5 * math.pi, "time-of-flight tilt out of range"
    out += (f'      <site name="tof" pos="{vec([c * MM for c in v["tof_position_mm"]])}" '
            f'zaxis="{vec((math.cos(tilt), 0.0, math.sin(tilt)))}" size="0.001"/>\n')
    return out


def _chassis_xml(v: Dict[str, Any], d: Dict[str, Any]) -> str:
    half = (0.5 * v["chassis_length_mm"] * MM, 0.5 * v["chassis_width_mm"] * MM, 0.5 * d["chassis_height_mm"] * MM)
    center_z = (v["chassis_ground_clearance_mm"] + 0.5 * d["chassis_height_mm"]) * MM
    assert all(h > 0.0 for h in half), "chassis must have size"
    assert v["chassis_ground_clearance_mm"] > 0.0, "chassis must clear the ground"
    return (
        f'      <freejoint name="root"/>\n'
        f'      <inertial pos="{vec([c * MM for c in d["chassis_com_mm"]])}" '
        f'mass="{num(d["masses_g"]["chassis"] * GRAM)}" diaginertia="{vec(d["chassis_inertia"])}"/>\n'
        f'      <geom name="chassis_geom" type="box" pos="{vec((v["chassis_center_x_mm"] * MM, 0, center_z))}" '
        f'size="{vec(half)}" rgba="0.35 0.35 0.4 1" group="{ROBOT_GEOM_GROUP}" {_contact_attrs(v, v["chassis_friction"])}/>\n'
        + _tip_supports_xml(v)
    )


def _actuators_xml(v: Dict[str, Any], d: Dict[str, Any]) -> str:
    drive = tread_drive(v)
    # The control is duty x battery voltage / reference voltage (robot._drive_treads); sag only lowers it.
    # The range only has to admit that; it must not do the scaling itself, so below the reference it stays 1.
    p_max = v["motor_power_max"] * max(1.0, v["battery_rest_voltage_v"] / v["motor_reference_voltage_v"])
    s_tq = v["servo_torque_max_mNm"] * 0.001
    assert p_max > 0.0 and s_tq > 0.0, "actuator limits must be positive"
    out = ""
    for side in ("left", "right"):
        joint = "wheel_fl_joint" if side == "left" else "wheel_fr_joint"
        out += (f'    <general name="{side}_wheel" joint="{joint}" gainprm="{num(drive["stall_Nm"])}" '
                f'biastype="affine" biasprm="0 0 {num(-drive["damping_Nms_per_rad"])}" '
                f'ctrlrange="{num(-p_max)} {num(p_max)}"/>\n')
    lo, hi = math.radians(v["head_angle_min_deg"]), math.radians(v["head_angle_max_deg"])
    out += (f'    <position name="head" joint="head_joint" kp="{num(v["head_servo_kp_mNm_per_rad"] * 0.001)}" '
            f'kv="{num(v["head_servo_kv_mNms_per_rad"] * 0.001)}" ctrlrange="{num(lo)} {num(hi)}" '
            f'forcerange="{num(-s_tq)} {num(s_tq)}"/>\n')
    out += (f'    <position name="lift" joint="lift_joint" kp="{num(v["lift_servo_kp_mNm_per_rad"] * 0.001)}" '
            f'kv="{num(v["lift_servo_kv_mNms_per_rad"] * 0.001)}" '
            f'ctrlrange="{num(d["lift_angle_min"])} {num(d["lift_angle_max"])}" '
            f'forcerange="{num(-s_tq)} {num(s_tq)}"/>\n')
    return out


def _equality_xml(v: Dict[str, Any], d: Dict[str, Any]) -> str:
    """One tread turns both its wheels at the same surface speed; the lift fork stays level.

    The couplings are as stiff as the solver allows: a time constant of two timesteps.
    """
    stiff = f'solref="{num(2.0 * v["sim_timestep_ms"] * 0.001)} 1"'
    gear = num(d["rear_gear"])
    assert d["rear_gear"] > 0.0, "rear wheel gear must be positive"
    assert v["sim_timestep_ms"] > 0.0, "timestep must be positive"
    return (
        '  <equality>\n'
        f'    <joint name="left_tread" joint1="wheel_rl_joint" joint2="wheel_fl_joint" '
        f'polycoef="0 {gear} 0 0 0" {stiff}/>\n'
        f'    <joint name="right_tread" joint1="wheel_rr_joint" joint2="wheel_fr_joint" '
        f'polycoef="0 {gear} 0 0 0" {stiff}/>\n'
        f'    <joint name="lift_level" joint1="wrist_joint" joint2="lift_joint" polycoef="0 -1 0 0 0" {stiff}/>\n'
        '  </equality>\n'
    )


def _sensors_xml(v: Dict[str, Any], cube: Any = None) -> str:
    assert len(_cliff_positions(v)) == 4, "the robot has four cliff sensors"
    out = ""
    for name, _, _ in _cliff_positions(v):
        out += f'    <rangefinder name="cliff_{name}" site="cliff_{name}"/>\n'
    out += '    <gyro name="gyro" site="imu"/>\n    <accelerometer name="accel" site="imu"/>\n'
    if cube is not None:
        out += '    <accelerometer name="cube_accel" site="cube_chip"/>\n'
    assert out.count("<rangefinder") == 4, "expected four cliff range sensors"
    return out


def _scene_xml(v: Dict[str, Any], obstacles: Sequence[Sequence[float]]) -> str:
    """Table with real edges, a floor below it, and optional boxes (centre and size in mm)."""
    assert len(obstacles) <= MAX_OBSTACLES, "too many obstacles"
    half = (0.5 * v["table_length_mm"] * MM, 0.5 * v["table_width_mm"] * MM, 0.5 * v["table_thickness_mm"] * MM)
    assert all(h > 0.0 for h in half) and v["table_height_mm"] > v["table_thickness_mm"], "bad table size"
    out = (f'    <geom name="floor" type="plane" pos="0 0 {num(-v["table_height_mm"] * MM)}" size="0 0 1" '
           f'contype="0" conaffinity="1" rgba="0.5 0.5 0.5 1"/>\n'
           f'    <geom name="table" type="box" pos="0 0 {num(-half[2])}" size="{vec(half)}" '
           f'contype="0" conaffinity="1" rgba="0.8 0.7 0.55 1"/>\n')
    for index, box in enumerate(obstacles):
        assert len(box) == 6 and all(s > 0.0 for s in box[3:]), "obstacle is (cx, cy, cz, sx, sy, sz) in mm"
        out += (f'    <geom name="obstacle_{index}" type="box" pos="{vec([c * MM for c in box[:3]])}" '
                f'size="{vec([0.5 * s * MM for s in box[3:]])}" contype="0" conaffinity="1" '
                f'rgba="0.6 0.3 0.3 1"/>\n')
    return out


def build_mjcf(v: Dict[str, Any], obstacles: Sequence[Sequence[float]] = (), cube: Any = None) -> str:
    """Return the whole model, robot and scene, as one MJCF string; cube="lift" adds a carried cube."""
    assert cube in CUBE_MOUNTS, "a cube goes nowhere or in the lift"
    d = derive(v)
    gravity = -v["gravity_mmps2"] * MM
    assert gravity < 0.0, "gravity must point down"
    assert v["sim_timestep_ms"] > 0.0, "timestep must be positive"
    return (
        '<mujoco model="vector_simulacrum">\n'
        '  <compiler angle="radian" autolimits="true"/>\n'
        f'  <option timestep="{num(v["sim_timestep_ms"] * 0.001)}" gravity="0 0 {num(gravity)}" '
        'integrator="implicitfast" cone="elliptic">\n    <flag multiccd="enable"/>\n  </option>\n'
        '  <worldbody>\n'
        + _scene_xml(v, obstacles)
        + '    <body name="chassis" pos="0 0 0">\n'
        + _chassis_xml(v, d) + _sites_xml(v)
        + _wheels_xml(v).replace("    <", "      <").replace("    </", "      </")
        + _head_xml(v).replace("    <", "      <").replace("    </", "      </")
        + _lift_xml(v, d, cube).replace("    <", "      <").replace("    </", "      </")
        + '    </body>\n  </worldbody>\n'
        + _equality_xml(v, d) +
        '  <actuator>\n' + _actuators_xml(v, d) + '  </actuator>\n'
        '  <sensor>\n' + _sensors_xml(v, cube) + '  </sensor>\n'
        '</mujoco>\n'
    )

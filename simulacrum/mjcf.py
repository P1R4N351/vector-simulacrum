"""MJCF generator for the Vector simulacrum..

Every number in the generated model comes from the parameter table (through
the value map from params.values or params.sample) or is derived from those
numbers here. The table is in millimetres and grams; MJCF is in metres,
kilograms and radians. No MuJoCo import: the output is a string.

Robot frame: origin on the ground under the centre of the front axle,
x forward, y left, z up. The table top is the plane z = 0.
"""

from __future__ import annotations

import copy
import csv
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple
import xml.etree.ElementTree as ET

from . import dock as dock_model

MM = 0.001
GRAM = 0.001
MAX_OBSTACLES = 16
CLIFF_NAMES = ("front_left", "front_right", "back_left", "back_right")
# The robot's own geoms are in this group and the scene's in group 0, so that the time-of-flight
# sensor's rays (robot.py) can be cast at the scene alone. Contact does not use groups.
ROBOT_GEOM_GROUP = 1
# Where the model may carry a light cube: nowhere (the default) or in the lift.
CUBE_MOUNTS = (None, "lift", "free")
DOCK_MOUNTS = dock_model.DOCK_MOUNTS


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


def lift_integral_offset_rad(v: Dict[str, Any], error_sum: float) -> float:
    """How far the lift's integral term moves the servo's target (rad).

    The firmware's lift power is Kp x error + Ki x error sum (LIFTC:832-835). The model's servo
    stands for Kp x error, so the sum adds Ki / Kp x the sum to the angle the servo pulls toward.
    """
    kp, ki = v["lift_power_gains"][0], v["lift_power_gains"][1]
    assert kp > 0.0 and ki >= 0.0 and math.isfinite(error_sum), "bad lift gains or error sum"
    return ki / kp * error_sum


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


def _moving_part_points(v: Dict[str, Any], lower_link_offset: Any = None) -> List[Tuple[float, float, float]]:
    """(mass g, x mm, z mm) of every part except the chassis, lift down, head level. The head's mass sits at
    the centre of its box. With lower_link_offset (x, z of the lower links' pivot from the upper one, mm;
    the real body's parallelogram) half the arm mass is on each pair of links."""
    m = _part_masses(v)
    hx, _, hz = v["head_box_center_wrt_neck_mm"]
    a0 = lift_angle_of_height(v, v["lift_height_min_mm"])
    sx, _, sz = v["lift_base_position_mm"]
    arm_len = v["lift_arm_length_mm"]
    wrist = (sx + arm_len * math.cos(a0), sz + arm_len * math.sin(a0))
    fx, fz = _fork_center_wrt_wrist(v)
    arm_x, arm_z = 0.5 * (sx + wrist[0]), 0.5 * (sz + wrist[1])
    arm = [(m["arm"], arm_x, arm_z)]
    if lower_link_offset is not None:
        ox, oz = lower_link_offset
        arm = [(0.5 * m["arm"], arm_x, arm_z), (0.5 * m["arm"], arm_x + ox, arm_z + oz)]
    points = [
        (m["head"], v["neck_joint_position_mm"][0] + hx, v["neck_joint_position_mm"][2] + hz),
        *arm,
        (m["fork"], wrist[0] + fx, wrist[1] + fz),
        (2.0 * m["wheel"], wheel_x_mm(v)[0], v["wheel_front_radius_mm"]),
        (2.0 * m["wheel"], wheel_x_mm(v)[1], v["wheel_rear_radius_mm"]),
    ]
    assert all(p[0] > 0.0 for p in points), "non-positive part mass"
    assert len(points) == (5 if lower_link_offset is None else 6), "part list changed"
    return points


def derive(v: Dict[str, Any], lower_link_offset: Any = None) -> Dict[str, Any]:
    """Quantities computed from the table: chassis inertial, limits, gear ratio. lower_link_offset as in
    _moving_part_points, for the real body, so that the whole robot keeps the table's centre of mass."""
    masses = _part_masses(v)
    points = _moving_part_points(v, lower_link_offset)
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
    half, half_w = 0.5 * v["tread_center_spacing_mm"] * MM, 0.5 * v["tread_width_mm"] * MM
    rf, rr, base = v["wheel_front_radius_mm"], v["wheel_rear_radius_mm"], v["wheel_base_mm"]
    assert gap < min(rf, rr), "tip supports would have no size"
    driven = _tip_supports_driven(v)
    out = ""
    for name, x_mm, radius_mm in (("tip_front", 0.0, rf), ("tip_rear", -base, rr)):
        for side, y in (("l", half), ("r", -half)):
            if driven:
                # Its own hinge, geared to the tread (tip_supports_driven): the surface moves with the tread.
                out += (f'      <body name="{name}_{side}_body" pos="{vec((x_mm * MM, y, radius_mm * MM))}">\n'
                        f'        <joint name="{name}_{side}_joint" type="hinge" axis="0 1 0" armature="1e-09"/>\n'
                        f'        <geom name="{name}_{side}" type="cylinder" fromto="0 {num(-half_w)} 0 0 {num(half_w)} 0" '
                        f'size="{num((radius_mm - gap) * MM)}" mass="1e-05" rgba="0.15 0.15 0.15 0.3" '
                        f'group="{ROBOT_GEOM_GROUP}" {_contact_attrs(v, v["tread_friction"])}/>\n'
                        f'      </body>\n')
                continue
            out += (f'      <geom name="{name}_{side}" type="cylinder" fromto="{num(x_mm * MM)} {num(y - half_w)} '
                    f'{num(radius_mm * MM)} {num(x_mm * MM)} {num(y + half_w)} {num(radius_mm * MM)}" '
                    f'size="{num((radius_mm - gap) * MM)}" mass="0" rgba="0.15 0.15 0.15 0.3" '
                    f'group="{ROBOT_GEOM_GROUP}" {_contact_attrs(v, v["tread_friction"])}/>\n')
    return out


def _tip_supports_driven(v: Dict[str, Any]) -> bool:
    """True when the tip supports turn with their tread (parameter tip_supports_driven, 0 or 1).

    The real tread wraps each axle, so where it meets the table round an axle it is driven: a robot
    tipped back onto its rear axle can still push itself along. Fixed to the body, a support is a
    brake with tread friction, and above about 5.5 degrees of pitch it carries the rear of the robot."""
    flag = v.get("tip_supports_driven", 0)
    assert flag in (0, 1), "tip_supports_driven is 0 or 1"
    return flag == 1 and v["tread_contact_inset_mm"] > 0.0


def _tip_equality_xml(v: Dict[str, Any], stiff: str) -> str:
    """Gear each driven tip support to its tread's front wheel joint, at equal surface speed."""
    if not _tip_supports_driven(v):
        return ""
    gap = v["tip_support_clearance_mm"]
    out = ""
    for name, radius_mm in (("tip_front", v["wheel_front_radius_mm"]), ("tip_rear", v["wheel_rear_radius_mm"])):
        gear = num(v["wheel_front_radius_mm"] / (radius_mm - gap))
        for side, wheel in (("l", "wheel_fl_joint"), ("r", "wheel_fr_joint")):
            out += (f'    <joint name="{name}_{side}_tread" joint1="{name}_{side}_joint" joint2="{wheel}" '
                    f'polycoef="0 {gear} 0 0 0" {stiff}/>\n')
    return out


def _wheels_xml(v: Dict[str, Any]) -> str:
    half = 0.5 * v["tread_center_spacing_mm"]  # as built; tread_spacing_mm is the firmware's constant
    assert half > 0.5 * v["chassis_width_mm"], "treads overlap the chassis"
    rf, rr = v["wheel_front_radius_mm"], v["wheel_rear_radius_mm"]
    front, rear = wheel_x_mm(v)
    return (_wheel_xml(v, "wheel_fl", front, half, rf, True) + _wheel_xml(v, "wheel_fr", front, -half, rf, True)
            + _wheel_xml(v, "wheel_rl", rear, half, rr, False) + _wheel_xml(v, "wheel_rr", rear, -half, rr, False))


def _camera_optics(v: Dict[str, Any]) -> str:
    width, height = v["camera_resolution_px"]
    # Keep explicitly changed FOVs and other stream modes on their specified projection.
    if (width, height) == (800, 600):
        path = Path(__file__).resolve().parent.parent / "web" / "camera_calibration_vector2.json"
        cal = json.loads(path.read_text())
        if (v["camera_fov_vertical_deg"] == cal["reported_fov_y_deg"] and
                v["camera_fov_horizontal_deg"] == cal["reported_fov_x_deg"]):
            # MuJoCo's principalpixel is the displacement of the image centre from the optical axis.
            # Only the sensor aspect ratio matters when focal and principal offsets are in pixels.
            return (f'sensorsize="{num(width / height)} 1" focalpixel="{cal["fx_px"]} {cal["fy_px"]}" '
                    f'principalpixel="{num(width / 2 - cal["cx_px"])} {num(height / 2 - cal["cy_px"])}"')
    return f'fovy="{num(v["camera_fov_vertical_deg"])}"'


def _head_xml(v: Dict[str, Any]) -> str:
    lo, hi = math.radians(v["head_angle_min_deg"]), math.radians(v["head_angle_max_deg"])
    assert lo < 0.0 < hi, "head range must straddle level"
    bx, by, bz = (0.5 * s * MM for s in v["head_box_mm"])
    offset = v["head_box_center_wrt_neck_mm"]
    assert len(offset) == 3, "head box offset has three axes"
    # Written only when set, so a centred box gives the same text as before the offset existed.
    box_pos = f' pos="{vec([c * MM for c in offset])}"' if any(offset) else ""
    arm = v["servo_armature_gcm2"] * 1e-7
    cam, imu = v["head_camera_position_mm"], v["imu_position_mm"]
    res = v["camera_resolution_px"]
    assert len(cam) == 3 and len(imu) == 3 and len(res) == 2, "bad head vectors"
    # The IMU's frame is pitched against the head (positive is nose up, like the head joint, whose axis is -y).
    half = 0.5 * math.radians(v["imu_pitch_offset_deg"])
    pitch = math.radians(v["head_camera_pitch_down_deg"])
    camera_up = vec((math.sin(pitch), 0, math.cos(pitch)))
    imu_quat = (math.cos(half), 0.0, -math.sin(half), 0.0)
    assert abs(half) < 0.5, "IMU pitch offset is not a small angle"
    return (
        f'    <body name="head" pos="{vec([c * MM for c in v["neck_joint_position_mm"]])}">\n'
        f'      <joint name="head_joint" type="hinge" axis="0 -1 0" range="{num(lo)} {num(hi)}" '
        f'armature="{num(arm)}"/>\n'
        f'      <geom name="head_geom" type="box"{box_pos} size="{vec((bx, by, bz))}" '
        f'mass="{num(v["head_mass_g"] * GRAM)}" rgba="0.25 0.25 0.3 1" group="{ROBOT_GEOM_GROUP}" '
        f'{_contact_attrs(v, v["chassis_friction"])}/>\n'
        f'      <camera name="head_cam" pos="{vec([c * MM for c in cam])}" xyaxes="0 -1 0 {camera_up}" '
        f'{_camera_optics(v)} resolution="{int(res[0])} {int(res[1])}"/>\n'
        f'      <site name="imu" pos="{vec([c * MM for c in imu])}" quat="{vec(imu_quat)}" size="0.001"/>\n'
        f'    </body>\n'
    )


def _cube_chip_xml(v: Dict[str, Any]) -> str:
    chip = v["cube_chip_position_mm"]
    yaw = math.radians(v["cube_chip_yaw_deg"])
    assert len(chip) == 3 and all(abs(c) < 0.5 * v["cube_size_mm"] for c in chip)
    return (f'<site name="cube_chip" pos="{vec([c * MM for c in chip])}" '
            f'quat="{vec((math.cos(0.5 * yaw), 0, 0, math.sin(0.5 * yaw)))}" size="0.001"/>')


CubeBox = Tuple[str, Tuple[float, float, float], Tuple[float, float, float]]
MAX_CUBE_BOXES = 400


def _box_minus(centre: Sequence[float], half: Sequence[float], lo: Sequence[float],
               hi: Sequence[float]) -> List[Tuple[Tuple[float, ...], Tuple[float, ...]]]:
    """The parts of one box outside the region [lo, hi], as at most six disjoint boxes (centre, half)."""
    b_lo = [c - h for c, h in zip(centre, half)]
    b_hi = [c + h for c, h in zip(centre, half)]
    if any(b_hi[a] <= lo[a] or b_lo[a] >= hi[a] for a in range(3)):
        return [(tuple(centre), tuple(half))]
    out = []
    cur_lo, cur_hi = list(b_lo), list(b_hi)
    for a in range(3):
        if cur_lo[a] < lo[a]:
            piece_hi = list(cur_hi)
            piece_hi[a] = lo[a]
            out.append((list(cur_lo), piece_hi))
            cur_lo[a] = lo[a]
        if cur_hi[a] > hi[a]:
            piece_lo = list(cur_lo)
            piece_lo[a] = hi[a]
            out.append((piece_lo, list(cur_hi)))
            cur_hi[a] = hi[a]
    return [(tuple(0.5 * (p + q) for p, q in zip(pl, ph)), tuple(0.5 * (q - p) for p, q in zip(pl, ph)))
            for pl, ph in out if all(q - p > 1e-6 for p, q in zip(pl, ph))]


def cube_boxes(v: Dict[str, Any]) -> List[CubeBox]:
    """The free cube's contact boxes in mm on the 44 mm frame of cube_surface.csv (the SDK shell and
    rubber pads), with the lift pocket behind each vertical edge's notch carved out: |x| and |y| inside
    cube_notch_pocket_xy_mm, |z| up to cube_notch_pocket_half_z_mm. What remains over each notch is the
    two thin lips the fork's hooks catch (measured by evidence/cube-pickup/measure_notch.py)."""
    back, lip = v["cube_notch_pocket_xy_mm"]
    top = v["cube_notch_pocket_half_z_mm"]
    assert 0.0 < back < lip < 22.0 and 0.0 < top < 22.0, "the notch pocket must sit inside the cube"
    rows: List[CubeBox] = []
    with Path(__file__).with_name("cube_surface.csv").open() as source:
        reader = csv.reader(source)
        next(reader)
        for material, *dimensions in reader:
            n = [float(x) for x in dimensions]
            rows.append((material, (n[0], n[1], n[2]), (n[3], n[4], n[5])))
    assert rows, "cube_surface.csv has no boxes"
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            lo = (min(sx * back, sx * lip), min(sy * back, sy * lip), -top)
            hi = (max(sx * back, sx * lip), max(sy * back, sy * lip), top)
            rows = [(m, c, h) for m, centre, half in rows for c, h in _box_minus(centre, half, lo, hi)]
            assert len(rows) <= MAX_CUBE_BOXES, "carving the pockets split the cube into too many boxes"
    return rows


def _free_cube_xml(v: Dict[str, Any]) -> str:
    """SDK shell and rubber pads from the shelf-push fixture, with the lift pockets of the animators' cube
    (cube_boxes), centered 100 mm ahead of world zero."""
    side = v["cube_outer_mm"] * MM
    mass = v["cube_mass_g"] * GRAM
    tc = v["contact_time_constant_ms"] * 0.001
    assert side > 0 and mass > 0
    assert tc >= 2.0 * v["sim_timestep_ms"] * 0.001, "contact time constant below two timesteps"
    inertia = mass * side**2 / 6
    parts = [f'<body name="cube" pos="0.1 0 {num(side/2)}"><freejoint name="cube_root"/>',
             f'<inertial pos="0 0 0" mass="{num(mass)}" diaginertia="{vec((inertia,)*3)}"/>']
    for i, (material, centre, half) in enumerate(cube_boxes(v)):
        scaled = [float(n) * side / 44 for n in (*centre, *half)]
        attrs = ('priority="2" friction=".9 .005 .0001" solref=".008 1" rgba=".035 .035 .035 1"'
                 if material == "rubber"
                 else f'priority="1" friction=".5 .005 .0001" solref="{num(tc)} 1" rgba=".2 .2 .2 1"')
        parts.append(f'<geom name="cube_{material}_{i}" type="box" pos="{vec(scaled[:3])}" '
                     f'size="{vec(scaled[3:])}" contype="1" conaffinity="1" {attrs}/>')
    return "\n".join([*parts, _cube_chip_xml(v), "</body>"])


def _carried_cube_xml(v: Dict[str, Any]) -> str:
    """A light cube held in the lift, as the engine places a carried cube (cube_carry_offset_mm from the
    wrist, on the level fork's axes), with a site at its accelerometer. The fork's body is the parent, so the
    two never collide with each other; the cube touches the world like the fork does."""
    half = 0.5 * v["cube_outer_mm"]
    assert half > 0.0 and v["cube_mass_g"] > 0.0, "the cube must have size and mass"
    assert len(v["cube_carry_offset_mm"]) == 3, "the carry offset has three axes"
    return (
        f'        <body name="cube" pos="{vec([c * MM for c in v["cube_carry_offset_mm"]])}">\n'
        f'          <geom name="cube_geom" type="box" size="{vec((half * MM,) * 3)}" '
        f'mass="{num(v["cube_mass_g"] * GRAM)}" rgba="0.85 0.85 0.8 1" group="{ROBOT_GEOM_GROUP}" '
        f'{_contact_attrs(v, v["chassis_friction"])}/>\n'
        f'          {_cube_chip_xml(v)}\n'
        f'        </body>\n'
    )


def _fork_hooks_xml(v: Dict[str, Any]) -> str:
    """The barbed hook on each fork upright: the upright behind it (lift_fork_hook_upright_mm) and the
    hook in slices along x (lift_fork_hook_slices_mm), each [x0, x1, z bottom, z top] in the fork's frame.
    The slices' tops dip into a valley ahead of the upright and rise to the tip. Docking slides the hooks
    into the notches in the cube's front vertical edges; as the lift rises, the cube's lip over each notch
    drops into the valley and the tip rises into the pocket behind it, so the cube cannot slide forward off."""
    y_in, y_out = v["lift_fork_hook_y_mm"]
    boxes = [v["lift_fork_hook_upright_mm"], *v["lift_fork_hook_slices_mm"]]
    assert 0.0 < y_in < y_out, "the hook needs an inner and an outer side"
    assert 1 < len(boxes) <= 33, "the hook needs one to thirty-two slices"
    out = ""
    for k, (x0, x1, z_lo, z_hi) in enumerate(boxes):
        assert x0 < x1 and z_lo < z_hi, "a hook box needs length and height"
        assert k == 0 or x0 >= boxes[k - 1][1] - 1e-9, "hook slices must run forward without overlapping"
        half = vec((0.5 * (x1 - x0) * MM, 0.5 * (y_out - y_in) * MM, 0.5 * (z_hi - z_lo) * MM))
        part = "upright" if k == 0 else f"hook_{k - 1}"
        for name, sign in (("left", 1), ("right", -1)):
            centre = (0.5 * (x0 + x1) * MM, sign * 0.5 * (y_in + y_out) * MM, 0.5 * (z_lo + z_hi) * MM)
            out += (f'        <geom name="fork_{part}_{name}" type="box" pos="{vec(centre)}" size="{half}" '
                    f'mass="0" {_contact_attrs(v, v["chassis_friction"])}/>\n')
    return out


def _lift_xml(v: Dict[str, Any], d: Dict[str, Any], cube: Any = None) -> str:
    assert cube in CUBE_MOUNTS, "unknown cube placement"
    arm_len = v["lift_arm_length_mm"] * MM
    half_w = 0.5 * v["lift_xbar_width_mm"] * MM
    fx, fz = _fork_center_wrt_wrist(v)
    half_x = 0.5 * (v["lift_front_wrt_wrist_mm"] - v["lift_back_wrt_wrist_mm"]) * MM
    half_z = 0.5 * v["lift_xbar_height_mm"] * MM
    arm = v["servo_armature_gcm2"] * 1e-7
    assert d["lift_angle_min"] < d["lift_angle_max"], "lift range is empty"
    assert arm_len > 0.0 and half_w > 0.0, "lift must have size"
    # Side plates contact objects above the crossbar; leave the opening clear.
    # The existing mass term already represents the entire fork assembly.
    x, y, z = v["lift_fork_side_center_mm"]
    size = vec([0.5 * n * MM for n in v["lift_fork_side_size_mm"]])
    sides = "".join(
        f'        <geom name="fork_side_{name}" type="box" pos="{vec((x * MM, sign * y * MM, z * MM))}" '
        f'size="{size}" mass="0" {_contact_attrs(v, v["chassis_friction"])}/>\n'
        for name, sign in (("left", 1), ("right", -1))
    ) + _fork_hooks_xml(v)
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
        f'{sides}      </body>\n'
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
    l_tq = v["lift_servo_torque_max_mNm"] * 0.001
    assert p_max > 0.0 and s_tq > 0.0 and l_tq > 0.0, "actuator limits must be positive"
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
    # The integral term moves the lift's target past either end of its travel (robot.py); the joint's
    # own range still stops the arm there, as the real lift's end stops do.
    reach = lift_integral_offset_rad(v, v["lift_error_sum_max"])
    out += (f'    <position name="lift" joint="lift_joint" kp="{num(v["lift_servo_kp_mNm_per_rad"] * 0.001)}" '
            f'kv="{num(v["lift_servo_kv_mNms_per_rad"] * 0.001)}" '
            f'ctrlrange="{num(d["lift_angle_min"] - reach)} {num(d["lift_angle_max"] + reach)}" '
            f'forcerange="{num(-l_tq)} {num(l_tq)}"/>\n')
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
        + _tip_equality_xml(v, stiff) +
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


def shelf_survey():
    return json.loads(Path(__file__).with_name("p2_shelf.json").read_text())


def _shelf_mesh(thickness_mm):
    """Triangular prisms preserve each surveyed edge, including a clipped corner."""
    outline = shelf_survey()["outline_mm"]
    assets, geoms = [], []
    for i, a in enumerate(outline):
        b = outline[(i + 1) % len(outline)]
        vertices = [n * MM for z in (0, -thickness_mm)
                    for x, y in ((0, 0), a, b) for n in (x, y, z)]
        name = f"shelf_{i}"
        assets.append(f'<mesh name="{name}" vertex="{vec(vertices)}"/>')
        geoms.append(f'<geom name="{name}" type="mesh" mesh="{name}" '
                     'contype="0" conaffinity="1" rgba="0.28 0.29 0.31 1"/>')
    return "<asset>" + "".join(assets) + "</asset>", "".join(geoms)


def _scene_xml(v: Dict[str, Any], obstacles: Sequence[Sequence[float]], shelf_geoms: str = "") -> str:
    """Table with real edges, a floor below it, and optional boxes (centre and size in mm)."""
    assert len(obstacles) <= MAX_OBSTACLES, "too many obstacles"
    half = (0.5 * v["table_length_mm"] * MM, 0.5 * v["table_width_mm"] * MM, 0.5 * v["table_thickness_mm"] * MM)
    assert all(h > 0.0 for h in half) and v["table_height_mm"] > v["table_thickness_mm"], "bad table size"
    out = (f'    <geom name="floor" type="plane" pos="0 0 {num(-v["table_height_mm"] * MM)}" size="0 0 1" '
           f'contype="0" conaffinity="1" rgba="0.5 0.5 0.5 1"/>\n'
           + (shelf_geoms or f'    <geom name="table" type="box" pos="0 0 {num(-half[2])}" size="{vec(half)}" '
           f'contype="0" conaffinity="1" rgba="0.8 0.7 0.55 1"/>\n'))
    for index, box in enumerate(obstacles):
        assert len(box) == 6 and all(s > 0.0 for s in box[3:]), "obstacle is (cx, cy, cz, sx, sy, sz) in mm"
        out += (f'    <geom name="obstacle_{index}" type="box" pos="{vec([c * MM for c in box[:3]])}" '
                f'size="{vec([0.5 * s * MM for s in box[3:]])}" contype="0" conaffinity="1" '
                f'rgba="0.6 0.3 0.3 1"/>\n')
    return out


def _dock_geom_attrs(role: str) -> str:
    """Contact and visual attributes for one role in dock_surface.csv."""
    assert role in ("ramp", "guide", "shell", "pin"), f"unknown dock role {role!r}"
    if role == "pin":
        return 'contype="0" conaffinity="0" rgba="0.78 0.63 0.33 1"'
    if role == "guide":
        return 'contype="0" conaffinity="1" priority="2" condim="3" friction="0.05 0.005 0.0001" rgba="0.18 0.19 0.21 1"'
    return 'contype="0" conaffinity="1" rgba="0.18 0.19 0.21 1"'


def _dock_xml(dock: Any) -> str:
    """Return `<body name="dock">...</body>` when requested, else empty string."""
    pose = dock_model.resolve_dock_pose(dock)
    if pose is None:
        return ""
    x_mm, y_mm, yaw_deg = pose
    yaw = math.radians(yaw_deg)
    quat = vec((math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)))
    geoms = []
    for box in dock_model.load_surface_boxes():
        pos = vec([c * MM for c in box.center_mm])
        half = vec([c * MM for c in box.half_mm])
        bquat = vec(box.quat)
        geoms.append(f'      <geom name="dock_{box.part}" type="box" pos="{pos}" quat="{bquat}" '
                     f'size="{half}" {_dock_geom_attrs(box.role)}/>\n')
    pin_l = vec([c * MM for c in dock_model.PIN_LEFT_MM])
    pin_r = vec([c * MM for c in dock_model.PIN_RIGHT_MM])
    marker = vec([c * MM for c in dock_model.MARKER_POS_MM])
    return (
        f'    <body name="dock" pos="{vec((x_mm * MM, y_mm * MM, 0.0))}" quat="{quat}">\n'
        + "".join(geoms)
        + f'      <site name="dock_pin_l" pos="{pin_l}" size="0.001"/>\n'
        + f'      <site name="dock_pin_r" pos="{pin_r}" size="0.001"/>\n'
        + f'      <site name="dock_marker" pos="{marker}" zaxis="-1 0 0" size="0.002"/>\n'
        + '    </body>\n'
    )


def build_mjcf(v: Dict[str, Any], obstacles: Sequence[Sequence[float]] = (), cube: Any = None,
               body: Any = None, *, dock: Any = None, environment: str = "table") -> str:
    """Return the whole model, robot and scene, as one MJCF string; cube="lift" adds a carried cube.

    body=None draws the robot as the table's boxes and cylinders. body=<manifest from
    simulacrum.body_mesh.load> gives it the real outer shape from Anki's rig: convex collision
    pieces and visual meshes on the chassis, head, both pairs of lift links and the fork."""
    assert cube in CUBE_MOUNTS, "unknown cube placement"
    gravity = -v["gravity_mmps2"] * MM
    assert gravity < 0.0, "gravity must point down"
    assert v["sim_timestep_ms"] > 0.0, "timestep must be positive"
    if environment not in ("table", "p2_shelf"):
        raise ValueError(f"unknown environment {environment}")
    shelf_assets, shelf_geoms = "", ""
    if environment == "p2_shelf":
        if dock is not None:
            raise ValueError("the shelf survey already places the dock")
        dock = shelf_survey()["dock_pose_mm"]
        shelf_assets, shelf_geoms = _shelf_mesh(v["table_thickness_mm"])
    dock_body = _dock_xml(dock)
    if body is not None:
        lower, upper = body["frames"]["lift_lower"]["origin_mm"], v["lift_base_position_mm"]
        d = derive(v, (lower[0] - upper[0], lower[2] - upper[2]))
        return _build_mesh_body(v, d, obstacles, cube, body, shelf_assets, shelf_geoms, dock_body)
    d = derive(v)
    return (
        '<mujoco model="vector_simulacrum">\n'
        '  <compiler angle="radian" autolimits="true"/>\n'
        f'  <option timestep="{num(v["sim_timestep_ms"] * 0.001)}" gravity="0 0 {num(gravity)}" '
        'integrator="implicitfast" cone="elliptic">\n    <flag multiccd="enable"/>\n  </option>\n'
        + (f'  <visual><global offwidth="{int(v["camera_resolution_px"][0])}" '
           f'offheight="{int(v["camera_resolution_px"][1])}"/></visual>\n'
           if tuple(v["camera_resolution_px"]) == (800, 600) else '')
        + shelf_assets + '  <worldbody>\n'
        + _scene_xml(v, obstacles, shelf_geoms)
        + dock_body
        + '    <body name="chassis" pos="0 0 0">\n'
        + _chassis_xml(v, d) + _sites_xml(v)
        + _wheels_xml(v).replace("    <", "      <").replace("    </", "      </")
        + _head_xml(v).replace("    <", "      <").replace("    </", "      </")
        + _lift_xml(v, d, cube).replace("    <", "      <").replace("    </", "      </")
        + '    </body>\n' + (_free_cube_xml(v) if cube == 'free' else '') + '  </worldbody>\n'
        + _equality_xml(v, d) +
        '  <actuator>\n' + _actuators_xml(v, d) + '  </actuator>\n'
        '  <sensor>\n' + _sensors_xml(v, cube) + '  </sensor>\n'
        '</mujoco>\n'
    )


def _prefix_element(element: ET.Element, prefix: str) -> ET.Element:
    """Deep-copy an MJCF element and prefix all body, joint, and actuator identifiers."""
    assert prefix and isinstance(element, ET.Element), "prefix and Element required"
    cloned = copy.deepcopy(element)
    for node in cloned.iter():
        for key in ("name", "joint", "joint1", "joint2"):
            val = node.get(key)
            if val is not None:
                node.set(key, prefix + val)
    return cloned


def _set_group_masks(body: ET.Element, contype: str, conaffinity: str) -> None:
    """Assign contact group masks to all collidable geoms under a body element."""
    for geom in body.iter("geom"):
        if geom.get("contype", "1") != "0":
            geom.set("contype", contype)
            geom.set("conaffinity", conaffinity)


def build_rescue_mjcf(v: Dict[str, Any], with_cube: bool = True, exclude_robot_contacts: bool = False) -> str:
    """Build a canonical two-robot (rescuer + patient_) MJCF scene on the open table."""
    root = ET.fromstring(build_mjcf(v, (), cube="free" if with_cube else None))
    world = root.find("worldbody")
    assert world is not None, "missing worldbody in base MJCF"
    rescuer = world.find("body[@name='chassis']")
    assert rescuer is not None, "missing rescuer chassis in base MJCF"
    patient = _prefix_element(rescuer, "patient_")
    world.append(patient)
    for geom in world.findall("geom"):
        geom.set("conaffinity", "7" if with_cube else "3")
    cube_body = world.find("body[@name='cube']")
    if cube_body is not None:
        _set_group_masks(cube_body, "4", "3")
    _set_group_masks(rescuer, "1", "6" if with_cube else "2")
    _set_group_masks(patient, "2", "5" if with_cube else "1")
    for section in ("equality", "actuator"):
        parent = root.find(section)
        assert parent is not None, f"missing {section} in base MJCF"
        parent.extend([_prefix_element(entry, "patient_") for entry in list(parent)])
    if with_cube:
        equality = root.find("equality")
        assert equality is not None, "missing equality section for carry_weld"
        ET.SubElement(equality, "weld", name="carry_weld", body1="lift_fork",
                      body2="cube", relpose="0.0335 0 -0.0125 1 0 0 0", active="false")
    if exclude_robot_contacts:
        contact = root.find("contact")
        if contact is None:
            contact = ET.SubElement(root, "contact")
        r_names = sorted(b.get("name") for b in rescuer.iter("body") if b.get("name"))
        p_names = sorted(b.get("name") for b in patient.iter("body") if b.get("name"))
        for r_name in r_names:
            for p_name in p_names:
                ET.SubElement(contact, "exclude", body1=r_name, body2=p_name)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode") + "\n"


# ---- the real outer shape (simulacrum/body_mesh.py) -------------------------------------------

MESH_PARTS = ("chassis", "head", "lift_upper", "lift_lower", "fork")
# A mass-carrying primitive that no longer touches anything: the rig's pieces take its contact.
_CARRIER = 'contype="0" conaffinity="0" group="3" rgba="0 0 0 0"'


# A visual mesh from a build that predates looks (body_mesh.LOOKS) is drawn in one flat grey.
_PLAIN_VISUAL = 'rgba="0.18 0.18 0.2 1"'
_LOOK_KEYS = ("rgba", "specular", "shininess", "reflectance", "emission")


def _mesh_assets_xml(body: Dict[str, Any]) -> str:
    folder = Path(body["dir"])
    assert folder.is_dir() and set(body["parts"]) == set(MESH_PARTS), "body manifest is incomplete"
    out = "  <asset>\n"
    looks = body.get("looks", {})
    for name in sorted(looks):
        look = looks[name]
        assert set(_LOOK_KEYS) <= set(look) and len(look["rgba"]) == 4, f"look {name} is incomplete"
        out += (f'    <material name="rig_{name}" rgba="{" ".join(num(c) for c in look["rgba"])}" '
                f'specular="{num(look["specular"])}" shininess="{num(look["shininess"])}" '
                f'reflectance="{num(look["reflectance"])}" emission="{num(look["emission"])}"/>\n')
    for part in MESH_PARTS:
        entry = body["parts"][part]
        for k, item in enumerate(entry["collision"]):
            out += f'    <mesh name="{part}_hull_{k}" file="{folder / item["file"]}" scale="0.001 0.001 0.001"/>\n'
        assert entry["visual"], f"{part} has no visual mesh"
        for k, item in enumerate(entry["visual"]):
            out += (f'    <mesh name="{part}_visual_{k}" file="{folder / item["file"]}" '
                    f'scale="0.001 0.001 0.001" inertia="shell"/>\n')
    return out + "  </asset>\n"


def _mesh_geoms_xml(v: Dict[str, Any], body: Dict[str, Any], part: str, indent: str) -> str:
    """One part's collision pieces (massless, contact as the table says) and its visual meshes."""
    entry = body["parts"][part]
    friction = {"chassis": v["chassis_friction"], "tread": v["tread_friction"]}
    out = ""
    for k, item in enumerate(entry["collision"]):
        assert item["friction"] in friction, "unknown friction class in the body manifest"
        out += (f'{indent}<geom name="{part}_hull_{k}" type="mesh" mesh="{part}_hull_{k}" mass="0" '
                f'rgba="0.4 0.4 0.45 0.25" group="{ROBOT_GEOM_GROUP}" '
                f'{_contact_attrs(v, friction[item["friction"]])}/>\n')
    for k, item in enumerate(entry["visual"]):
        look = item.get("look")
        assert look is None or look in body.get("looks", {}), f"{part}: look {look!r} missing from the manifest"
        drawn = _PLAIN_VISUAL if look is None else f'material="rig_{look}"'
        out += (f'{indent}<geom name="{part}_visual_{k}" type="mesh" mesh="{part}_visual_{k}" mass="0" contype="0" '
                f'conaffinity="0" group="2" {drawn}/>\n')
    return out


def _as_carrier(xml: str, geom: str) -> str:
    """Strip a primitive geom's contact and drawing, keeping its mass and its name."""
    start = xml.index(f'<geom name="{geom}"')
    end = xml.index("/>", start)
    tag = xml[start:end]
    for attr in ("contype", "conaffinity", "priority", "condim", "friction", "solref", "group", "rgba"):
        idx = tag.find(f' {attr}="')
        if idx >= 0:
            tag = tag[:idx] + tag[tag.index('"', tag.index('"', idx) + 1) + 1:]
    return xml[:start] + tag + " " + _CARRIER + xml[end:]


def _lower_link_xml(v: Dict[str, Any], d: Dict[str, Any], body: Dict[str, Any]) -> str:
    """The lift's lower pair of links: a second hinge below the first, held at the lift's angle by an
    equality (the real lift is a parallelogram). It carries half the arm mass; the upper pair the rest."""
    origin = body["frames"]["lift_lower"]["origin_mm"]
    arm_len = v["lift_arm_length_mm"] * MM
    assert len(origin) == 3 and arm_len > 0.0, "lower link frame missing"
    return (
        f'      <body name="lift_lower" pos="{vec([c * MM for c in origin])}">\n'
        f'        <joint name="lift_lower_joint" type="hinge" axis="0 -1 0" '
        f'range="{num(d["lift_angle_min"])} {num(d["lift_angle_max"])}"/>\n'
        f'        <geom name="lift_lower_geom" type="box" pos="{vec((0.5 * arm_len, 0, 0))}" '
        f'size="{vec((0.5 * arm_len, 0.5 * v["lift_xbar_width_mm"] * MM, 0.002))}" '
        f'mass="{num(0.5 * d["masses_g"]["arm"] * GRAM)}" {_CARRIER}/>\n'
        + _mesh_geoms_xml(v, body, "lift_lower", "        ")
        + '      </body>\n'
    )


def _build_mesh_body(v: Dict[str, Any], d: Dict[str, Any], obstacles: Sequence[Sequence[float]],
                     cube: Any, body: Dict[str, Any], shelf_assets: str, shelf_geoms: str, dock_body: str) -> str:
    """The parametric model with the rig's shape swapped in for the robot's contact and drawing."""
    chassis = _as_carrier(_chassis_xml(v, d), "chassis_geom") + _mesh_geoms_xml(v, body, "chassis", "      ")
    head = _as_carrier(_head_xml(v), "head_geom")
    head = head.replace("    </body>\n", _mesh_geoms_xml(v, body, "head", "      ") + "    </body>\n")
    lift = _lift_xml(v, d, cube)
    # The splices below anchor on text the primitive builders write; a changed builder must fail here.
    assert head.count("    </body>\n") == 1, "head builder changed shape"
    assert lift.count('      <body name="lift_fork"') == 1, "lift builder changed shape"
    lift = lift[:lift.index('        <geom name="fork_side_left"')] + "      </body>\n    </body>\n"
    assert lift.count(f'mass="{num(d["masses_g"]["arm"] * GRAM)}"') >= 1, "arm mass not found in the lift"
    lift = _as_carrier(_as_carrier(lift, "lift_fork_geom"), "lift_arm_geom")
    lift = lift.replace(f'mass="{num(d["masses_g"]["arm"] * GRAM)}"', f'mass="{num(0.5 * d["masses_g"]["arm"] * GRAM)}"', 1)
    lift = lift.replace('      <body name="lift_fork"', _mesh_geoms_xml(v, body, "lift_upper", "      ")
                        + '      <body name="lift_fork"', 1)
    lift = lift.replace("      </body>\n    </body>\n", _mesh_geoms_xml(v, body, "fork", "        ")
                        + "      </body>\n    </body>\n", 1)
    stiff = f'solref="{num(2.0 * v["sim_timestep_ms"] * 0.001)} 1"'
    equality = _equality_xml(v, d).replace(
        "  </equality>\n", f'    <joint name="lift_lower_follow" joint1="lift_lower_joint" joint2="lift_joint" '
        f'polycoef="0 1 0 0 0" {stiff}/>\n  </equality>\n')
    indent = lambda xml: xml.replace("    <", "      <").replace("    </", "      </")
    return (
        '<mujoco model="vector_simulacrum">\n'
        '  <compiler angle="radian" autolimits="true"/>\n'
        f'  <option timestep="{num(v["sim_timestep_ms"] * 0.001)}" gravity="0 0 {num(-v["gravity_mmps2"] * MM)}" '
        'integrator="implicitfast" cone="elliptic">\n    <flag multiccd="enable"/>\n  </option>\n'
        + _mesh_assets_xml(body) + shelf_assets +
        '  <worldbody>\n'
        + _scene_xml(v, obstacles, shelf_geoms) + dock_body
        + '    <body name="chassis" pos="0 0 0">\n'
        + chassis + _sites_xml(v) + indent(_wheels_xml(v)) + indent(head) + indent(lift)
        + _lower_link_xml(v, d, body)
        + '    </body>\n' + (_free_cube_xml(v) if cube == 'free' else '') + '  </worldbody>\n'
        + equality +
        '  <actuator>\n' + _actuators_xml(v, d) + '  </actuator>\n'
        '  <sensor>\n' + _sensors_xml(v, cube) + '  </sensor>\n'
        '</mujoco>\n'
    )

"""SDK-shaped robot layer over the MuJoCo model..

Takes the commands the real Vector takes through the SDK, in the same units
(wheel speeds in mm/s, distances in mm, angles in degrees, lift height in mm)
and reports what the SDK reports, at the robot's state rate. It also models
the robot's own cliff reaction, with a switch for the two control levels:

* CONTROL_DEFAULT: the reaction is on. A cliff stops the treads, cancels the
  running action, takes control away, and after a pause backs the robot off.
* CONTROL_OVERRIDE: the reaction is off. The robot keeps driving and falls.

The treads are driven as the firmware drives them (simulacrum.drive): motor
power from its speed loop, on speeds measured from encoder counts. Gyro
and accelerometer pass through the IMU chip's low-pass filter at every
physics step (simulacrum.imu_filter) and are read once per control tick.
Gyro, accelerometer and forward range carry noise drawn from a seeded
generator. With cube="lift" the robot carries a light cube, and the cube's
own accelerometer and app (simulacrum.cube) run at every physics step on the
specific force at its chip; the lift then starts at carry height.

The pose it reports is the simulator's true pose. The real robot reports an
estimate from odometry; that error is not modelled here.
"""

from __future__ import annotations

import collections
import math
import random
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import mujoco
import numpy as np

from . import cube as cube_model
from . import imu_filter, mjcf, params
from .drive import TreadController, TreadEncoder, whole_mmps
from .hazards import Hazards
from .prox import ProxReading, ProxSensor

CONTROL_DEFAULT = "default"
CONTROL_OVERRIDE = "override"
RESULT_COMPLETED = "completed"
RESULT_CLIFF = "cancelled_cliff"
RESULT_NO_CONTROL = "no_control"
RESULT_TIMEOUT = "timeout"
RESULT_OFF_GROUND = "aborted_off_ground"
MAX_ACTION_S = 60.0  # code-loop bound for one blocking action, not a model of the robot
STATE_HISTORY = 8192


@dataclass(frozen=True)
class RobotState:
    """One robot_state message. Field names follow the SDK's RobotState."""

    time_s: float
    x_mm: float
    y_mm: float
    z_mm: float
    pose_angle_rad: float
    pose_pitch_rad: float
    left_wheel_speed_mmps: float
    right_wheel_speed_mmps: float
    head_angle_rad: float
    lift_height_mm: float
    prox_distance_mm: float
    prox_found_object: bool
    prox_is_lift_in_fov: bool
    accel_mmps2: Tuple[float, float, float]
    gyro_radps: Tuple[float, float, float]
    cliff_detected: bool
    picked_up: bool
    falling: bool
    wheels_moving: bool
    has_control: bool
    prox_signal_quality: float = 0.0
    prox_unobstructed: bool = True
    lift_in_pos: bool = True
    head_in_pos: bool = True


@dataclass(frozen=True)
class ActionResult:
    result: str
    moved_mm: float
    turned_deg: float
    ticks: int


def _clamp(value: float, low: float, high: float) -> float:
    assert low <= high, "empty clamp range"
    return min(max(value, low), high)


def _toward(current: float, target: float, max_step: float) -> float:
    assert max_step >= 0.0, "step must not be negative"
    return current + _clamp(target - current, -max_step, max_step)


def _profile_step(current: float, target: float, rate: float, cap: float, accel: float, dt: float) -> Tuple[float, float]:
    """One tick of a speed-and-acceleration profile toward a target: speed up at accel, never above cap,
    and slow down so as to stop on the target. Returns the new position and speed."""
    assert cap > 0.0 and accel > 0.0 and dt > 0.0 and rate >= 0.0, "profile needs positive speed, acceleration and step"
    remaining = abs(target - current)
    if remaining == 0.0:
        return current, 0.0
    rate = min(cap, rate + accel * dt, math.sqrt(2.0 * accel * remaining))
    nxt = _toward(current, target, rate * dt)
    return nxt, (0.0 if nxt == target else rate)


def lift_error_sum_step(v: Dict[str, Any], error_sum: float, error_rad: float, final: bool) -> float:
    """One control tick of the firmware lift's integral term (LIFTC:858-892).

    Holding its final target within lift_angle_tol_deg, the lift adds nothing to the sum and only
    bleeds it while its power (Kp x error + Ki x sum) is above lift_power_in_position_max. Off
    target, or while the target is still moving, the error goes on the sum. The sum is clipped to
    lift_error_sum_max either way.
    """
    assert math.isfinite(error_sum) and math.isfinite(error_rad), "bad lift error"
    kp, ki = v["lift_power_gains"][0], v["lift_power_gains"][1]
    cap = v["lift_error_sum_max"]
    assert cap > 0.0 and v["lift_error_sum_decay"] > 0.0, "the error sum needs a positive cap and decay"
    if abs(error_rad) < math.radians(v["lift_angle_tol_deg"]) and final:
        power = kp * error_rad + ki * error_sum
        if abs(power) > v["lift_power_in_position_max"]:
            error_sum -= v["lift_error_sum_decay"] * (1.0 if power > 0.0 else -1.0)
    else:
        error_sum += error_rad
    return _clamp(error_sum, -cap, cap)


class VectorSim:
    """A simulated Vector on a table. One instance is one robot and one scene."""

    def __init__(self, values: Optional[Dict[str, Any]] = None, control_level: str = CONTROL_DEFAULT,
                 start_xy_mm: Tuple[float, float] = (0.0, 0.0), heading_deg: float = 0.0,
                 obstacles: Sequence[Sequence[float]] = (), head_deg: float = 0.0,
                 lift_mm: Optional[float] = None, noise_seed: int = 0, cube: Optional[str] = None,
                 cube_app: str = "last", body: Optional[Dict[str, Any]] = None, *, body_range_clock: Optional[bool] = None, charger_contact: bool = False,
                 environment: str = "table") -> None:
        assert control_level in (CONTROL_DEFAULT, CONTROL_OVERRIDE), "unknown control level"
        assert isinstance(noise_seed, int), "noise seed must be an int"
        assert cube in mjcf.CUBE_MOUNTS, "unknown cube placement"
        self._noise = random.Random(noise_seed)
        self.v = dict(values) if values is not None else params.values(params.load())
        if cube == "lift" and lift_mm is None:
            lift_mm = self.v["lift_height_max_mm"]  # FW:79 LIFT_HEIGHT_CARRY, the cube carrying height
        # The time-of-flight sensor draws from its own stream, so its readings do not move the IMU's noise.
        self._prox_sensor = ProxSensor(self.v, noise_seed + 1)
        self.body_range_clock = environment == "p2_shelf" if body_range_clock is None else body_range_clock
        self.charger_contact = charger_contact
        self._range_body_phase_us = 0
        self._range_body_ticks = 0
        self._range_sample: Optional[Tuple[float, float]] = None
        self._tof_hits_mm: List[float] = []
        self.d = mjcf.derive(self.v)
        self.control_level = control_level
        self.dt = self.v["control_period_ms"] * 0.001
        # Existing replay builders accept the original two-argument scene interface.
        self._shelf_outline = None
        if environment != "table":
            xml = mjcf.build_mjcf(self.v, obstacles, cube, body, environment=environment)
            self._shelf_outline = mjcf.shelf_survey()["outline_mm"]
        elif body is not None:
            xml = mjcf.build_mjcf(self.v, obstacles, cube, body)
        else:
            xml = mjcf.build_mjcf(self.v, obstacles) if cube is None else mjcf.build_mjcf(self.v, obstacles, cube)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self._index_model()
        self._init_commands()
        # The cube draws noise from its own stream, so carrying one does not move the robot's IMU noise.
        self.cube = (cube_model.CubeAccelerometer(self.v, float(self.model.opt.timestep), seed=noise_seed + 2, app=cube_app)
                     if cube is not None else None)
        if self.cube is not None:
            self.cube.handling_cube = cube == "lift"
        assert (self.cube is None) == (self._cube_adr < 0), "cube sensor and model must agree"
        self.hazards = Hazards(self.v)
        self.states: collections.deque = collections.deque(maxlen=STATE_HISTORY)
        self._place(start_xy_mm, heading_deg)
        self._pose_servos(head_deg, lift_mm)
        self._settle()

    @classmethod
    def randomised(cls, seed: int, **kwargs: Any) -> "VectorSim":
        """A robot whose ranged parameters are drawn inside their ranges from one seed."""
        assert isinstance(seed, int), "seed must be an int"
        kwargs.setdefault("noise_seed", seed)
        return cls(values=params.sample(params.load(), seed), **kwargs)

    # ---- set-up ---------------------------------------------------------

    def _index_model(self) -> None:
        """Look up every address once."""
        m = self.model
        sensor = lambda name: int(m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)])
        joint = lambda name: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        act = lambda name: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        self._cliff_adr = [sensor("cliff_" + name) for name in mjcf.CLIFF_NAMES]
        self._gyro_adr, self._accel_adr = sensor("gyro"), sensor("accel")
        self._imu_adr = np.array([self._accel_adr + i for i in range(3)] + [self._gyro_adr + i for i in range(3)])
        assert len(self._imu_adr) == imu_filter.CHANNELS, "the filter takes accelerometer then gyro"
        cube_sensor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "cube_accel")
        self._cube_adr = int(m.sensor_adr[cube_sensor]) if cube_sensor >= 0 else -1
        self._tof_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tof")
        assert self._tof_site >= 0, "time-of-flight site missing from the model"
        scene_only = [0] * 6
        scene_only[0] = 1  # group 0 is the scene; the robot's own geoms are in mjcf.ROBOT_GEOM_GROUP
        assert mjcf.ROBOT_GEOM_GROUP != 0, "the robot's geoms must not share the scene's group"
        self._scene_only = np.array(scene_only, dtype=np.uint8)
        self._q = {n: int(m.jnt_qposadr[joint(n + "_joint")]) for n in ("head", "lift", "wrist")}
        lower = joint("lift_lower_joint")  # present only with the real body shape
        self._lower_q = int(m.jnt_qposadr[lower]) if lower >= 0 else -1
        self._dof = {n: int(m.jnt_dofadr[joint(n + "_joint")]) for n in ("wheel_fl", "wheel_fr", "head", "lift")}
        self._wheel_q = [int(m.jnt_qposadr[joint(n + "_joint")]) for n in ("wheel_fl", "wheel_fr")]
        self._act = {n: act(n) for n in ("left_wheel", "right_wheel", "head", "lift")}
        assert all(a >= 0 for a in self._act.values()), "actuator missing from the model"
        assert m.nu == 4 and len(self._cliff_adr) == 4, "model shape differs from the layer's"

    def _init_commands(self) -> None:
        v = self.v
        self.tick = 0
        self.has_control = True
        self.reactions = 0
        self._cancelled = False
        self._in_action = False
        self._wheel_target = [0.0, 0.0]
        self._wheel_accel = [0.0, 0.0]
        self._wheel_cmd = [0.0, 0.0]
        self._applied = [0.0, 0.0]
        self._encoders = [TreadEncoder(v), TreadEncoder(v)]
        self._controllers = [TreadController(v), TreadController(v)]
        assert v["motor_reference_voltage_v"] > 0.0 and v["battery_rest_voltage_v"] > 0.0, "voltages must be positive"
        assert v["battery_sag_v_per_power"] >= 0.0, "sag must not be negative"
        self.battery_volts = v["battery_rest_voltage_v"]
        assert v["tread_heat_tau_s"] > 0.0 and len(v["tread_heat_loss_rate_per_s"]) == 2, "heat needs a time and two rates"
        self.tread_heat = [0.0, 0.0]
        self._measured = (0.0, 0.0)
        self._imu_sample: Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = None
        self._imu_filter = imu_filter.ImuFilter(v["accel_filter_cutoff_hz"], v["gyro_filter_cutoff_hz"],
                                                v["imu_filter_second_pole"], float(self.model.opt.timestep))
        self._head_target = self._head_cmd = 0.0
        self._lift_target = self._lift_cmd = self.d["lift_angle_min"]
        self._head_speed, self._lift_speed = v["head_speed_default_radps"], v["lift_speed_default_radps"]
        self._head_accel, self._lift_accel = v["head_accel_default_radps2"], v["lift_accel_default_radps2"]
        self._servo_rate = [0.0, 0.0]
        self._pending_head_command = None
        self._head_in_pos = True
        self._head_in_tolerance_since = None
        self._lift_in_pos = True
        self._lift_in_tolerance_since = None
        self._lift_servo_cmd = self._lift_cmd  # the lift target the motor acts on this tick, before the integral term
        self._lift_error_sum = 0.0  # the firmware's angleErrorSum_ (rad x control ticks)
        self._wheel_delay = int(round(v["command_delay_ms"] / v["control_period_ms"]))
        self._servo_delay = int(round(v["servo_command_delay_ms"] / v["control_period_ms"]))
        assert self._wheel_delay >= 0 and self._servo_delay >= 0, "command delays must not be negative"
        slots = max(self._wheel_delay, self._servo_delay) + 1
        self._delay = [[0.0, 0.0, self._head_cmd, self._lift_cmd, None] for _ in range(slots)]
        self._delay_at = 0
        self._react_phase, self._react_ticks, self._react_dir = "idle", 0, -1.0
        self._react_from = (0.0, 0.0)
        self._state_ticks = int(round(v["state_period_ms"] / v["control_period_ms"]))
        assert self._state_ticks >= 1, "state period shorter than the control period"

    def _place(self, start_xy_mm: Tuple[float, float], heading_deg: float) -> None:
        assert len(start_xy_mm) == 2 and math.isfinite(heading_deg), "bad start pose"
        yaw = math.radians(heading_deg)
        qpos = self.data.qpos
        qpos[0], qpos[1], qpos[2] = start_xy_mm[0] * mjcf.MM, start_xy_mm[1] * mjcf.MM, 0.0
        qpos[3], qpos[4], qpos[5], qpos[6] = math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)
        qpos[self._q["lift"]] = self.d["lift_angle_min"]
        qpos[self._q["wrist"]] = -self.d["lift_angle_min"]
        if self._lower_q >= 0:
            qpos[self._lower_q] = self.d["lift_angle_min"]
        self.data.ctrl[self._act["lift"]] = self._lift_servo_cmd = self.d["lift_angle_min"]
        mujoco.mj_forward(self.model, self.data)
        self._yaw_prev = self._yaw_total = self._yaw()
        self._dock_travel_mm = [0.0, 0.0]

    def _pose_servos(self, head_deg: float, lift_mm: Optional[float]) -> None:
        """Start with the head and lift already at a given angle and height (clamped to the limits)."""
        v = self.v
        head = math.radians(_clamp(head_deg, v["head_angle_min_deg"], v["head_angle_max_deg"]))
        lift = mjcf.lift_angle_of_height(v, lift_mm if lift_mm is not None else v["lift_height_min_mm"])
        assert math.isfinite(head) and math.isfinite(lift), "bad head or lift start"
        qpos = self.data.qpos
        qpos[self._q["head"]], qpos[self._q["lift"]], qpos[self._q["wrist"]] = head, lift, -lift
        if self._lower_q >= 0:
            qpos[self._lower_q] = lift
        self._accepted_head_target = self._head_target = self._head_cmd = head
        self._lift_target = self._lift_cmd = lift
        self._servo_rate = [0.0, 0.0]
        for slot in self._delay:
            slot[2], slot[3] = head, lift
        self.data.ctrl[self._act["head"]], self.data.ctrl[self._act["lift"]] = head, lift
        self._lift_servo_cmd, self._lift_error_sum = lift, 0.0
        mujoco.mj_forward(self.model, self.data)

    def _settle(self) -> None:
        """Let the robot come to rest, then start the clock from zero."""
        ticks = int(round(self.v["settle_time_ms"] / self.v["control_period_ms"]))
        assert 0 <= ticks <= 2000, "settle time out of range"
        for _ in range(ticks):
            mujoco.mj_step(self.model, self.data, self.d["substeps"])
        self.data.time = 0.0
        self._reset_imu_filter()
        if self.cube is not None:
            self.cube.reset(self._cube_force_g(), 0.0)
        self._yaw_prev = self._yaw_total = self._yaw()
        for encoder, travel in zip(self._encoders, self._tread_travel_mm()):
            encoder.update(travel)
            encoder.counts, encoder.speed_mmps = 0, 0.0
        self._publish_state()

    # ---- reading the model ----------------------------------------------

    def _yaw(self) -> float:
        w, x, y, z = (float(c) for c in self.data.qpos[3:7])
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _pitch(self) -> float:
        w, x, y, z = (float(c) for c in self.data.qpos[3:7])
        return math.asin(_clamp(2.0 * (x * z - w * y), -1.0, 1.0))

    def pose_mm(self) -> Tuple[float, float, float]:
        """True position of the robot origin, in mm, in the table frame."""
        qpos = self.data.qpos
        return (float(qpos[0]) / mjcf.MM, float(qpos[1]) / mjcf.MM, float(qpos[2]) / mjcf.MM)

    def heading_total_rad(self) -> float:
        """Heading with whole turns kept, so a 360 degree turn reads 2 pi."""
        return self._yaw_total

    def _wheel_speeds(self) -> Tuple[float, float]:
        """Signed tread surface speeds from the model, mm/s."""
        radius = self.v["wheel_front_radius_mm"]
        qvel = self.data.qvel
        return (float(qvel[self._dof["wheel_fl"]]) * radius, float(qvel[self._dof["wheel_fr"]]) * radius)

    def _tread_travel_mm(self) -> Tuple[float, float]:
        """Signed travel of each tread surface so far, mm."""
        radius = self.v["wheel_front_radius_mm"]
        qpos = self.data.qpos
        return (float(qpos[self._wheel_q[0]]) * radius, float(qpos[self._wheel_q[1]]) * radius)

    def _reported_wheel_speeds(self) -> Tuple[float, float]:
        """Wheel speeds as the firmware measures them.

        The magnitude comes from encoder counts. With one encoder channel the sign comes
        from the command. The firmware keeps the value in an integer.
        """
        v = self.v
        out = []
        for encoder in self._encoders:
            speed = encoder.speed_mmps * encoder.direction
            out.append(whole_mmps(speed) if v["wheel_speed_reported_whole_mmps"] else speed)
        assert len(out) == 2, "two treads"
        return out[0], out[1]

    def cliff_ranges_mm(self) -> List[float]:
        """Range under each cliff sensor in mm (FL, FR, BL, BR); -1 when nothing is below."""
        out = [float(self.data.sensordata[adr]) for adr in self._cliff_adr]
        assert len(out) == 4, "four cliff sensors"
        return [r / mjcf.MM if r >= 0.0 else -1.0 for r in out]

    def _cast_tof(self) -> List[float]:
        """Cast the sensor's fan of rays at the scene (not at the robot itself).
        Per ray: the distance in mm to what it hit, or -1 for nothing."""
        origin = np.array(self.data.site_xpos[self._tof_site], dtype=float)
        frame = np.array(self.data.site_xmat[self._tof_site], dtype=float).reshape(3, 3)
        geom_id = np.zeros(1, dtype=np.int32)
        hits: List[float] = []
        for local in self._prox_sensor.rays:
            dist = float(mujoco.mj_ray(self.model, self.data, origin, frame @ np.array(local), self._scene_only, 1, -1, geom_id))
            hits.append(dist / mjcf.MM if dist >= 0.0 else -1.0)
        self._tof_hits_mm = list(hits)
        assert len(hits) == len(self._prox_sensor.rays), "one hit per ray of the fan"
        return hits

    def _prox(self) -> ProxReading:
        """What the time-of-flight sensor reports now (simulacrum/prox.py), from the lift height the robot reports."""
        lift_mm = self._reported_head_and_lift()[1]
        if not self.body_range_clock:
            return self._prox_sensor.read(self._cast_tof(), lift_mm, self._pitch())
        if self._range_sample is None:
            self._range_sample = self._prox_sensor.acquire(self._cast_tof(), lift_mm)
        return self._prox_sensor.report(self._range_sample, lift_mm, self._pitch())

    def _advance_range_clock(self) -> None:
        """Syscon keeps the elapsed body ticks when tread drive or charger contact changes."""
        ticks, self._range_body_phase_us = divmod(self._range_body_phase_us + round(self.dt * 1e6), 5120)
        driven = any(abs(controller.power) > 0.1 for controller in self._controllers)
        target = 6 if driven else (200 if self.charger_contact else 12)
        for _ in range(ticks):
            self._range_body_ticks += 1
            if self._range_body_ticks >= target:
                self._range_body_ticks = 0
                self._range_sample = self._prox_sensor.acquire(self._cast_tof(), self._reported_head_and_lift()[1])

    def _reset_imu_filter(self) -> None:
        """Start the IMU filter at the present reading: the robot has been still long enough."""
        self._imu_filter.reset(np.array(self.data.sensordata[self._imu_adr], dtype=float))

    def _cube_force_g(self) -> np.ndarray:
        """Specific force at the carried cube's chip, on the chip's axes, in g (1 g reads +1 on a vertical z)."""
        assert self._cube_adr >= 0, "no cube in the model"
        g = self.v["gravity_mmps2"] * mjcf.MM
        return np.array(self.data.sensordata[self._cube_adr:self._cube_adr + 3], dtype=float) / g

    def _physics_step(self) -> None:
        """One control tick of physics: the IMU chip filters at every step, not once per tick, and so does a
        carried cube's."""
        for _ in range(self.d["substeps"]):
            mujoco.mj_step(self.model, self.data)
            self._imu_filter.update(np.array(self.data.sensordata[self._imu_adr], dtype=float))
            if self.cube is not None:
                self.cube.step(float(self.data.time), self._cube_force_g())

    def _sample_imu(self) -> None:
        """Read the filtered gyro and accelerometer once, apply this robot's scale, bias and noise, clip to the
        part's range. Scale and bias pass through the chip's filter unchanged (it is linear with unit gain at
        rest), so applying them after it is the same; the noise is the robot's output noise as recorded."""
        v, gauss, out = self.v, self._noise.gauss, self._imu_filter.output
        mg = 0.001 * v["gravity_mmps2"]
        # Stock HAL converts the signed chip word using MAX_16BIT_POSITIVE.
        accel_step = v["accel_range_g"] * 9810.0 / 32767
        g_max = math.radians(v["gyro_range_dps"])
        accel_axes = []
        for i in range(3):
            value = (float(out[i]) / mjcf.MM * v["accel_scale"][i] + mg * v["accel_bias_mg"][i]
                             + gauss(0.0, mg * v["accel_noise_rms_mg"]))
            count = _clamp(math.floor(value / accel_step + 0.5), -32768, 32767)
            accel_axes.append(count * accel_step)
        accel = tuple(accel_axes)
        # Stock corrects the Z chip sensitivity by 0.989; true simulated rates
        # are already in corrected units. Only the reported count spacing changes.
        gyro_axes = []
        for i, sensitivity in enumerate((1.0, 1.0, 0.989)):
            step = g_max * sensitivity / 32768
            rate = float(out[3 + i]) + gauss(0.0, math.radians(v["gyro_noise_rms_dps"]))
            count = _clamp(math.floor(rate / step + 0.5), -32768, 32767)
            gyro_axes.append(_clamp(count * step + math.radians(v["gyro_bias_dps"][i]), -g_max, g_max))
        gyro = tuple(gyro_axes)
        assert len(accel) == 3 and len(gyro) == 3, "IMU vectors have three axes"
        self._imu_sample = (accel, gyro)

    def _imu(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """The IMU sample of this control tick."""
        if self._imu_sample is None:
            self._sample_imu()
        assert self._imu_sample is not None, "no IMU sample"
        return self._imu_sample

    def reported_now(self) -> Dict[str, float]:
        """What the robot would report this control tick: the signals a recording of it carries."""
        left, right = self._reported_wheel_speeds()
        head, lift = self._reported_head_and_lift()
        _, gyro = self._imu()
        assert len(gyro) == 3, "gyro has three axes"
        return {"wl": left, "wr": right, "gyro_z_dps": math.degrees(gyro[2]), "head_deg": math.degrees(head),
                "lift_mm": lift}

    def lift_height_mm(self) -> float:
        return mjcf.lift_height_of_angle(self.v, float(self.data.qpos[self._q["lift"]]))

    def head_angle_rad(self) -> float:
        return float(self.data.qpos[self._q["head"]])

    def _reported_head_and_lift(self) -> Tuple[float, float]:
        """Head angle (rad) and lift height (mm) in whole encoder counts."""
        v = self.v
        head_step, lift_step = v["head_rad_per_encoder_count"], v["lift_rad_per_encoder_count"]
        assert head_step > 0.0 and lift_step > 0.0, "encoder steps must be positive"
        head = round(self.head_angle_rad() / head_step) * head_step
        # The firmware resets the lift encoder at its calibrated low position.
        low = self.d["lift_angle_min"]
        counts = round((float(self.data.qpos[self._q["lift"]]) - low) / lift_step)
        lift_angle = low + counts * lift_step
        return head, mjcf.lift_height_of_angle(v, lift_angle)

    def _publish_state(self) -> None:
        x, y, z = self.pose_mm()
        left, right = self._reported_wheel_speeds()
        prox = self._prox()
        accel, gyro = self._imu()
        head, lift = self._reported_head_and_lift()
        moving = (abs(left) + abs(right)) > self.v["wheel_stopped_threshold_mmps"]
        self.state = RobotState(
            time_s=self.tick * self.dt, x_mm=x - self._dock_travel_mm[0], y_mm=y - self._dock_travel_mm[1], z_mm=z, pose_angle_rad=self._yaw(),
            pose_pitch_rad=self._pitch(), left_wheel_speed_mmps=left, right_wheel_speed_mmps=right,
            head_angle_rad=head, lift_height_mm=lift, prox_distance_mm=prox.distance_mm,
            prox_found_object=prox.found_object, prox_is_lift_in_fov=prox.is_lift_in_fov,
            prox_signal_quality=prox.signal_quality, prox_unobstructed=prox.unobstructed,
            accel_mmps2=accel, gyro_radps=gyro, cliff_detected=self.hazards.cliff_any,
            picked_up=self.hazards.picked_up, falling=self.hazards.falling, wheels_moving=moving,
            has_control=self.has_control, lift_in_pos=self._lift_in_pos, head_in_pos=self._head_in_pos)
        self.states.append(self.state)

    def sim_truth(self) -> Dict[str, Any]:
        """Facts the real robot does not report, for tests and for episode termination."""
        v = self.v
        x, y, z = self.pose_mm()
        margin = min(0.5 * v["table_length_mm"] - abs(x), 0.5 * v["table_width_mm"] - abs(y))
        if self._shelf_outline:
            outline = self._shelf_outline
            inside = False
            margin = float("inf")
            for (ax, ay), (bx, by) in zip(outline, outline[1:] + outline[:1]):
                dx, dy = bx - ax, by - ay
                t = max(0., min(1., ((x-ax)*dx + (y-ay)*dy)/(dx*dx + dy*dy)))
                margin = min(margin, math.hypot(x-ax-t*dx, y-ay-t*dy))
                if (ay > y) != (by > y) and x < ax + (y-ay)*dx/dy:
                    inside = not inside
            if not inside:
                margin = -margin
        assert v["table_thickness_mm"] > 0.0, "table thickness must be positive"
        return {"fallen": z < -v["table_thickness_mm"], "edge_margin_mm": margin, "z_mm": z,
                "cliff_flags": list(self.hazards.cliff), "wheel_speeds_mmps": self._wheel_speeds(),
                "encoder_counts": (self._encoders[0].counts, self._encoders[1].counts),
                "motor_power": (self._controllers[0].power, self._controllers[1].power),
                "tof_hits_mm": list(self._tof_hits_mm)}

    # ---- the control tick -----------------------------------------------

    def _ramp_commands(self) -> None:
        """Move the commanded values toward their targets at the commanded rates."""
        for i in range(2):
            accel = self._wheel_accel[i]
            step = accel * self.dt if accel > 0.0 else float("inf")
            self._wheel_cmd[i] = self._wheel_target[i] if accel <= 0.0 else _toward(
                self._wheel_cmd[i], self._wheel_target[i], step)
        head_cap = min(self._head_speed, self.v["head_speed_max_radps"])
        lift_cap = min(self._lift_speed, self.v["lift_speed_max_radps"])
        self._head_cmd, self._servo_rate[0] = _profile_step(self._head_cmd, self._head_target, self._servo_rate[0],
                                                            head_cap, self._head_accel, self.dt)
        self._lift_cmd, self._servo_rate[1] = _profile_step(self._lift_cmd, self._lift_target, self._servo_rate[1],
                                                            lift_cap, self._lift_accel, self.dt)

    def _delayed_command(self) -> List[float]:
        """Queue this tick's SDK command; return the wheel speeds issued command_delay_ms ago
        and the head and lift targets issued servo_command_delay_ms ago."""
        size = len(self._delay)
        slot = self._delay[self._delay_at]
        slot[0], slot[1], slot[2], slot[3] = self._wheel_cmd[0], self._wheel_cmd[1], self._head_cmd, self._lift_cmd
        slot[4] = self._pending_head_command
        self._pending_head_command = None
        wheels = self._delay[(self._delay_at - self._wheel_delay) % size]
        servos = self._delay[(self._delay_at - self._servo_delay) % size]
        if servos[4] is not None:
            self._accept_head_target(servos[4])
        self._delay_at = (self._delay_at + 1) % size
        out = [wheels[0], wheels[1], servos[2], servos[3]]
        assert len(out) == 4, "delay slot shape"
        return out

    def _start_reaction(self) -> None:
        """The robot's own response to a cliff: stop now, lose SDK control, back off after a pause."""
        v = self.v
        self.reactions += 1
        self.has_control = False
        self._cancelled = True
        front = self.hazards.cliff[0] or self.hazards.cliff[1]
        self._react_dir = -1.0 if front else 1.0
        self._react_phase = "pause"
        self._react_ticks = int(round(v["cliff_reaction_pause_ms"] / v["control_period_ms"]))
        self._wheel_target, self._wheel_cmd = [0.0, 0.0], [0.0, 0.0]
        self._head_cmd = self.head_angle_rad()
        self._head_target = self._head_cmd
        self._accept_head_target(self._head_cmd)
        self._pending_head_command = None
        self._lift_target = self._lift_cmd = float(self.data.qpos[self._q["lift"]])
        self._servo_rate = [0.0, 0.0]
        for slot in self._delay:
            slot[4] = None
            slot[0], slot[1], slot[2], slot[3] = 0.0, 0.0, self._head_cmd, self._lift_cmd
        assert self._react_ticks >= 0, "cliff reaction pause must not be negative"

    def _reaction_wheels(self) -> Tuple[float, float]:
        """Wheel speeds the reaction commands this tick."""
        v = self.v
        if self._react_phase == "pause":
            self._react_ticks -= 1
            if self._react_ticks <= 0:
                self._react_phase = "move"
                self._react_from = self.pose_mm()[:2]
                budget_s = 3.0 * v["cliff_reaction_backup_mm"] / v["cliff_reaction_backup_speed_mmps"]
                self._react_ticks = int(math.ceil(budget_s / self.dt))
            return 0.0, 0.0
        x, y, _ = self.pose_mm()
        gone = math.hypot(x - self._react_from[0], y - self._react_from[1])
        self._react_ticks -= 1
        if gone >= v["cliff_reaction_backup_mm"] or self._react_ticks <= 0:
            self._react_phase = "idle"
            return 0.0, 0.0
        speed = self._react_dir * v["cliff_reaction_backup_speed_mmps"]
        return speed, speed

    def _maybe_react(self) -> None:
        """Firmware rule: stop when a cliff newly appears while driving and not already stopping."""
        if self.control_level != CONTROL_DEFAULT or self._react_phase != "idle":
            return
        left, right = self._measured
        driving = (abs(left) + abs(right)) > self.v["wheel_stopped_threshold_mmps"]
        stopping = self._applied[0] == 0.0 and self._applied[1] == 0.0
        if self.hazards.cliff_new and driving and not stopping:
            self._start_reaction()

    def _write_controls(self) -> None:
        delayed = self._delayed_command()
        if self._react_phase != "idle":
            left, right = self._reaction_wheels()
        elif self.has_control and self._in_action:
            left, right = self._wheel_cmd[0], self._wheel_cmd[1]
        elif self.has_control:
            left, right = delayed[0], delayed[1]
        else:
            left, right = 0.0, 0.0
        ctrl = self.data.ctrl
        ctrl[self._act["head"]] = delayed[2]
        self._write_lift(delayed[3])
        self._applied = [left, right]
        self._drive_treads(left, right)

    def _write_lift(self, command: float) -> None:
        """Aim the lift servo at this tick's target plus the integral term, then update the error sum.

        The firmware computes its power from the sum before this tick adds to it (LIFTC:834, 888).
        The target is final once the commanded profile has reached the target last set.
        """
        error = command - float(self.data.qpos[self._q["lift"]])
        self.data.ctrl[self._act["lift"]] = command + mjcf.lift_integral_offset_rad(self.v, self._lift_error_sum)
        self._lift_servo_cmd = command
        self._lift_error_sum = lift_error_sum_step(self.v, self._lift_error_sum, error, command == self._lift_target)

    def _drive_treads(self, left_mmps: float, right_mmps: float) -> None:
        """The firmware's speed loop: commanded and measured speed in, motor power out.

        The power is a PWM duty (hal_motors.cpp MotorSetPower), so the motor sees duty x battery voltage.
        The actuator's gain is the stall torque at motor_reference_voltage_v and its damping (back EMF)
        does not depend on the voltage, so the duty is scaled by the battery's voltage over that reference.
        The battery's voltage sags with the treads' summed duty of the last tick.
        Each tread also loses drive as it heats: a first-order heat state follows its |duty| with time
        constant tread_heat_tau_s, and the drive is scaled by 1 - rate x tau x heat (motor_heat_fit.py).
        """
        v = self.v
        measured = self._reported_wheel_speeds()
        load = abs(self._controllers[0].power) + abs(self._controllers[1].power)
        self.battery_volts = v["battery_rest_voltage_v"] - v["battery_sag_v_per_power"] * load
        assert self.battery_volts > 0.0, "battery voltage sagged to nothing"
        scale = self.battery_volts / v["motor_reference_voltage_v"]
        ctrl = self.data.ctrl
        duties = (self._controllers[0].update(left_mmps, measured[0]), self._controllers[1].update(right_mmps, measured[1]))
        assert max(abs(duties[0]), abs(duties[1])) <= v["motor_power_max"], "power outside its limit"
        tau, rates = v["tread_heat_tau_s"], v["tread_heat_loss_rate_per_s"]
        keep = [1.0 - rates[i] * tau * self.tread_heat[i] for i in range(2)]
        assert min(keep) > 0.0, "a tread lost all its drive to heat"
        ctrl[self._act["left_wheel"]] = duties[0] * scale * keep[0]
        ctrl[self._act["right_wheel"]] = duties[1] * scale * keep[1]
        for i in range(2):
            self.tread_heat[i] += (abs(duties[i]) - self.tread_heat[i]) * min(1.0, self.dt / tau)

    def _set_head_target(self, target: float) -> None:
        self._head_target = self._pending_head_command = target

    def _accept_head_target(self, target: float) -> None:
        if target != self._accepted_head_target or not self._head_in_pos or abs(target - self.head_angle_rad()) >= math.radians(self.v["head_angle_tol_deg"]):
            self._head_in_pos = False
            self._head_in_tolerance_since = None
        self._accepted_head_target = target

    def _update_head_status(self) -> None:
        """HeadController latches in-position after the final target stays within tolerance for >200 ms."""
        if self._head_in_pos:
            return
        angle = float(self.data.qpos[self._q["head"]])
        final_command = self.data.ctrl[self._act["head"]] == self._accepted_head_target
        if not final_command or abs(angle - self._accepted_head_target) >= math.radians(self.v["head_angle_tol_deg"]):
            self._head_in_tolerance_since = None
        elif self._head_in_tolerance_since is None:
            self._head_in_tolerance_since = self.tick
        elif (self.tick - self._head_in_tolerance_since) * self.dt > 0.200:
            self._head_in_pos = True

    def _update_lift_status(self) -> None:
        """LiftController latches in-position after the final target stays within tolerance for >100 ms."""
        if self._lift_in_pos:
            return
        angle = float(self.data.qpos[self._q["lift"]])
        final_command = self._lift_servo_cmd == self._lift_target
        if not final_command or abs(angle - self._lift_target) >= math.radians(self.v["lift_angle_tol_deg"]):
            self._lift_in_tolerance_since = None
        elif self._lift_in_tolerance_since is None:
            self._lift_in_tolerance_since = self.tick
        elif (self.tick - self._lift_in_tolerance_since) * self.dt > 0.100:
            self._lift_in_pos = True

    def _tick_once(self) -> None:
        drive = (self._wheel_speeds() if self.v["encoder_tread_direction_sensed"] else
                 (self.data.ctrl[self._act["left_wheel"]], self.data.ctrl[self._act["right_wheel"]]))
        for encoder, travel, direction in zip(self._encoders, self._tread_travel_mm(), drive):
            encoder.update(travel, direction)
        self._measured = self._reported_wheel_speeds()
        self._sample_imu()
        accel, _ = self._imu()
        self.hazards.update(self.cliff_ranges_mm(), math.sqrt(sum(a * a for a in accel)))
        self._maybe_react()
        self._ramp_commands()
        self._write_controls()
        before = self.pose_mm() if self.charger_contact else None
        self._physics_step()
        if before is not None:
            after = self.pose_mm()
            self._dock_travel_mm[0] += after[0] - before[0]
            self._dock_travel_mm[1] += after[1] - before[1]
        self.tick += 1
        self._update_head_status()
        self._update_lift_status()
        if self.body_range_clock:
            self._advance_range_clock()
        yaw = self._yaw()
        self._yaw_total += math.atan2(math.sin(yaw - self._yaw_prev), math.cos(yaw - self._yaw_prev))
        self._yaw_prev = yaw
        if self.tick % self._state_ticks == 0:
            self._publish_state()

    def step(self, ticks: int = 1) -> RobotState:
        """Advance the robot by whole control ticks and return the latest state message."""
        assert isinstance(ticks, int) and 0 <= ticks <= int(MAX_ACTION_S / self.dt), "tick count out of range"
        for _ in range(ticks):
            self._tick_once()
        return self.state

    def advance(self, seconds: float) -> RobotState:
        assert 0.0 <= seconds <= MAX_ACTION_S, "advance time out of range"
        return self.step(int(round(seconds / self.dt)))

    # ---- SDK-shaped commands --------------------------------------------

    def request_control(self) -> bool:
        """Ask for control back. Refused while the robot is still reacting."""
        if self._react_phase != "idle":
            return False
        self.has_control = True
        self._cancelled = False
        return True

    def set_wheel_motors(self, left_mmps: float, right_mmps: float, left_accel_mmps2: float = 0.0,
                         right_accel_mmps2: float = 0.0) -> bool:
        """Set tread speeds. An acceleration of 0 means the speed is commanded at once.

        Not clamped to wheel_speed_max_mmps: the firmware applies that only in path following
        (steeringController.cpp:419 CheckWheelSpeedLimits), not in direct drive (ExecuteDirectDrive),
        and the robot reached 243-247 mm/s when sent 300 (REC2 run F). The motor sets the ceiling."""
        assert all(math.isfinite(x) for x in (left_mmps, right_mmps, left_accel_mmps2, right_accel_mmps2))
        if not self.has_control:
            return False
        self._wheel_target = [left_mmps, right_mmps]
        self._wheel_accel = [abs(left_accel_mmps2), abs(right_accel_mmps2)]
        return True

    def stop_all_motors(self) -> bool:
        if not self.has_control:
            return False
        self._wheel_target, self._wheel_accel = [0.0, 0.0], [0.0, 0.0]
        self._head_cmd = self.head_angle_rad()
        self._set_head_target(self._head_cmd)
        self._lift_target = self._lift_cmd = float(self.data.qpos[self._q["lift"]])
        self._retarget_lift()  # the firmware stops the lift by aiming it where it is (LIFTC:927-930, 458-474)
        self._lift_in_tolerance_since = None
        self._servo_rate = [0.0, 0.0]
        return True

    def _retarget_lift(self) -> None:
        """A new lift target. Arriving while the lift is in position, it clears the error sum, so a
        short move is not overpowered by the sum unwinding (LIFTC:566-573)."""
        if self._lift_in_pos:
            self._lift_error_sum = 0.0
        self._lift_in_pos = False

    def _interrupt(self) -> Optional[str]:
        """Why a running action must end now, or None.

        A cliff reaction cancels it. So does leaving the ground: the layer assumes
        the robot abandons an action once it reports falling or picked up.
        """
        if self._cancelled:
            return RESULT_CLIFF
        if self.hazards.picked_up or self.hazards.falling:
            return RESULT_OFF_GROUND
        return None

    def _begin_action(self) -> None:
        """An action is one SDK command: it reaches the robot after the command delay,
        and from then on the robot's own 200 Hz loop steers the treads with no further delay."""
        assert not self._in_action, "one action at a time"
        self._wheel_target, self._wheel_accel = [0.0, 0.0], [0.0, 0.0]
        for _ in range(self._wheel_delay):
            self._tick_once()
        self._in_action = True

    def command_head_angle(self, angle_deg: float) -> bool:
        """Set the head target and return at once (set_head_angle waits for the head)."""
        assert math.isfinite(angle_deg), "bad head angle"
        if not self.has_control:
            return False
        v = self.v
        self._head_speed, self._servo_rate[0] = v["head_speed_default_radps"], 0.0
        self._set_head_target(math.radians(_clamp(angle_deg, v["head_angle_min_deg"], v["head_angle_max_deg"])))
        return True

    def _start_lift(self, height_mm: float, speed_radps: Optional[float], duration_s: float) -> None:
        assert math.isfinite(duration_s) and duration_s >= 0.0, "bad lift duration"
        target = mjcf.lift_angle_of_height(self.v, height_mm)
        if target != self._lift_target or abs(target - self.data.qpos[self._q["lift"]]) >= math.radians(self.v["lift_angle_tol_deg"]):
            self._retarget_lift()
        self._lift_in_tolerance_since = None
        self._lift_target = target
        self._lift_speed = abs(speed_radps) if speed_radps else self.v["lift_speed_default_radps"]
        self._lift_accel = self.v["lift_accel_default_radps2"]
        self._servo_rate[1] = 0.0
        span = abs(self._lift_target - self._lift_cmd)
        if duration_s > 0 and span > 0:
            # Stock SetLiftHeight uses 10% acceleration, 80% cruise, 10% braking.
            # The area under that angular velocity trapezoid is the requested span.
            peak = span / (0.9 * duration_s)
            self._lift_speed = min(peak, self.v["lift_speed_max_radps"])
            self._lift_accel = min(10000.0, peak / (0.1 * duration_s))

    def command_lift_height(self, height_mm: float, duration_s: float = 0.0) -> bool:
        """Set the lift target and return at once (set_lift_height waits for the lift)."""
        assert math.isfinite(height_mm), "bad lift height"
        if not self.has_control:
            return False
        self._start_lift(height_mm, None, duration_s)
        return True

    def _budget_ticks(self, expected_s: float) -> int:
        assert expected_s >= 0.0, "expected time must not be negative"
        return int(min(MAX_ACTION_S, 3.0 * expected_s + 5.0) / self.dt)

    def _finish_action(self, result: str, start: Tuple[float, float, float], yaw0: float, tick0: int) -> ActionResult:
        """Stop the treads, wait for them to stop, and report what the action did."""
        if self.has_control:
            self._wheel_target, self._wheel_accel = [0.0, 0.0], [0.0, 0.0]
        for _ in range(int(1.0 / self.dt)):
            left, right = self._wheel_speeds()
            stopped = (abs(left) + abs(right)) <= self.v["wheel_stopped_threshold_mmps"]
            if (stopped and self._react_phase == "idle") or result == RESULT_OFF_GROUND:
                break
            self._tick_once()
        self._in_action = False
        x, y, _ = self.pose_mm()
        moved = math.hypot(x - start[0], y - start[1])
        assert result in (RESULT_COMPLETED, RESULT_CLIFF, RESULT_TIMEOUT, RESULT_OFF_GROUND), "unknown action result"
        return ActionResult(result, moved, math.degrees(self._yaw_total - yaw0), self.tick - tick0)

    def drive_straight(self, dist_mm: float, speed_mmps: float) -> ActionResult:
        """Drive a distance along the current heading. Negative distance is backwards."""
        assert math.isfinite(dist_mm) and math.isfinite(speed_mmps) and speed_mmps > 0.0, "bad drive request"
        if not self.has_control:
            return ActionResult(RESULT_NO_CONTROL, 0.0, 0.0, 0)
        v = self.v
        speed = min(speed_mmps, v["wheel_speed_max_mmps"])
        sign = 1.0 if dist_mm >= 0.0 else -1.0
        start, yaw0, tick0 = self.pose_mm(), self._yaw_total, self.tick
        self._begin_action()
        ux, uy = math.cos(self._yaw()), math.sin(self._yaw())
        result, v_cmd = RESULT_TIMEOUT, 0.0
        for _ in range(self._budget_ticks(abs(dist_mm) / speed)):
            stop = self._interrupt()
            if stop is not None:
                result = stop
                break
            x, y, _ = self.pose_mm()
            remaining = abs(dist_mm) - sign * ((x - start[0]) * ux + (y - start[1]) * uy)
            if remaining <= v["drive_done_mm"]:
                result = RESULT_COMPLETED
                break
            v_cmd = min(speed, v_cmd + v["wheel_accel_default_mmps2"] * self.dt,
                        math.sqrt(2.0 * v["wheel_decel_default_mmps2"] * remaining))
            steer = v["heading_hold_gain_mmps_per_rad"] * (yaw0 - self._yaw_total)
            self._wheel_target = [sign * v_cmd - steer, sign * v_cmd + steer]
            self._tick_once()
        return self._finish_action(result, start, yaw0, tick0)

    def turn_in_place(self, angle_deg: float, speed_dps: Optional[float] = None) -> ActionResult:
        """Turn by an angle; positive is counter-clockwise seen from above."""
        assert math.isfinite(angle_deg), "bad turn request"
        if not self.has_control:
            return ActionResult(RESULT_NO_CONTROL, 0.0, 0.0, 0)
        v = self.v
        rate = math.radians(speed_dps) if speed_dps is not None else v["point_turn_speed_default_radps"]
        rate = _clamp(abs(rate), 0.01, math.radians(v["body_rotation_speed_max_dps"]))
        accel, half = v["point_turn_accel_default_radps2"], 0.5 * v["tread_spacing_mm"]
        sign = 1.0 if angle_deg >= 0.0 else -1.0
        start, yaw0, tick0 = self.pose_mm(), self._yaw_total, self.tick
        self._begin_action()
        result, w_cmd = RESULT_TIMEOUT, 0.0
        for _ in range(self._budget_ticks(abs(math.radians(angle_deg)) / rate)):
            stop = self._interrupt()
            if stop is not None:
                result = stop
                break
            remaining = abs(math.radians(angle_deg)) - sign * (self._yaw_total - yaw0)
            if remaining <= math.radians(v["turn_tolerance_deg"]):
                result = RESULT_COMPLETED
                break
            w_cmd = min(rate, w_cmd + accel * self.dt, math.sqrt(2.0 * accel * remaining))
            self._wheel_target = [-sign * w_cmd * half, sign * w_cmd * half]
            self._tick_once()
        return self._finish_action(result, start, yaw0, tick0)

    def _servo_action(self, done: Callable[[int], bool], span: float, speed: float) -> ActionResult:
        """Wait for the head or lift to reach its target."""
        assert speed > 0.0, "servo speed must be positive"
        start, yaw0, tick0 = self.pose_mm(), self._yaw_total, self.tick
        result = RESULT_TIMEOUT
        for _ in range(self._budget_ticks(span / speed)):
            stop = self._interrupt()
            if stop is not None:
                result = stop
                break
            if done(self.tick - tick0):
                result = RESULT_COMPLETED
                break
            self._tick_once()
        x, y, _ = self.pose_mm()
        return ActionResult(result, math.hypot(x - start[0], y - start[1]),
                            math.degrees(self._yaw_total - yaw0), self.tick - tick0)

    def set_head_angle(self, angle_deg: float, speed_radps: Optional[float] = None) -> ActionResult:
        """Tilt the head; the angle is clamped to the robot's range."""
        assert math.isfinite(angle_deg), "bad head angle"
        if not self.has_control:
            return ActionResult(RESULT_NO_CONTROL, 0.0, 0.0, 0)
        v = self.v
        self._head_speed = abs(speed_radps) if speed_radps else v["head_speed_default_radps"]
        self._servo_rate[0] = 0.0
        target = math.radians(_clamp(angle_deg, v["head_angle_min_deg"], v["head_angle_max_deg"]))
        span = abs(target - self.head_angle_rad())
        self._set_head_target(target)
        tol = math.radians(v["head_angle_tol_deg"])
        assert tol > 0.0, "head tolerance must be positive"

        def settled(elapsed: int) -> bool:
            return (elapsed >= self._servo_delay and self._accepted_head_target == target
                    and self._head_in_pos and abs(self.head_angle_rad() - target) <= tol)

        return self._servo_action(settled, span, self._head_speed)

    def set_lift_height(self, height_mm: float, speed_radps: Optional[float] = None, duration_s: float = 0.0) -> ActionResult:
        """Raise or lower the lift; the height is clamped to the robot's range."""
        assert math.isfinite(height_mm), "bad lift height"
        if not self.has_control:
            return ActionResult(RESULT_NO_CONTROL, 0.0, 0.0, 0)
        v = self.v
        self._start_lift(height_mm, speed_radps, duration_s)
        target = self._lift_target
        read = lambda: float(self.data.qpos[self._q["lift"]])
        span = abs(target - read())
        tol = math.radians(v["lift_angle_tol_deg"])
        assert tol > 0.0, "lift tolerance must be positive"
        def settled(_: int) -> bool:
            return (span <= tol or self._lift_in_pos) and abs(read() - target) <= tol

        return self._servo_action(settled, span, self._lift_speed)

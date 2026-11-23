"""Speed-and-turn following task on the simulacrum..

One VectorTask is one robot on one table for one episode at a time. The
policy is given a forward speed and a turn rate to hold and answers with two
wheel speeds. No torch here: training wraps this, and the deployed policy
uses the same observation code, so the two cannot drift apart.

Policy step: one state period, read from the table (60 ms, measured on the
recording of the real robot). A command takes about that long to act, so a
faster policy would act on observations it does not have.

Observation: only what the real robot reports at its state rate, plus the
command and the policy's own last action. No pose, no true velocity, no
individual cliff sensors, no picked-up or falling flag.

Every episode draws a new robot: all ranged robot parameters, the table
size and the contact stiffness, inside the ranges of parameters.json.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

from . import params
from .robot import CONTROL_DEFAULT, CONTROL_OVERRIDE, VectorSim

OBS_NAMES = (
    "left_wheel_speed", "right_wheel_speed", "gyro_x", "gyro_y", "gyro_z",
    "accel_x", "accel_y", "accel_z", "cliff_detected", "prox_distance",
    "prox_found_object", "head_angle", "lift_height", "command_forward",
    "command_turn", "last_action_left", "last_action_right",
)
OBS_DIM = len(OBS_NAMES)
ACT_DIM = 2
RANDOMISED_KINDS = ("robot", "scene", "numeric")
# The integrator step is not drawn: it must divide the control period, and halving it doubles the cost.
NOT_RANDOMISED = ("sim_timestep_ms", "tip_supports_driven")  # the flag is model structure, 0 or 1
END_TIME = "time_limit"
END_REACTION = "cliff_reaction"
END_FALL = "fell"


@dataclass(frozen=True)
class TaskConfig:
    """Design choices of the task. These are not properties of the robot."""

    episode_s: float = 10.0
    command_resample_s: float = 4.0
    zero_command_prob: float = 0.1
    override_fraction: float = 0.25
    start_edge_margin_mm: float = 100.0
    sigma_forward_mmps: float = 40.0
    sigma_turn_dps: float = 30.0
    weight_forward: float = 1.0
    weight_turn: float = 0.5
    weight_action_rate: float = 0.05
    fall_penalty: float = 20.0
    randomise: bool = True
    body_range_clock: bool = False


@dataclass(frozen=True)
class StepResult:
    observation: Tuple[float, ...]
    reward: float
    terminated: bool
    truncated: bool
    end_reason: Optional[str]
    forward_error_mmps: float
    turn_error_dps: float


def observation_scales(v: Dict[str, Any]) -> Dict[str, float]:
    """Fixed divisors that bring each reported quantity to about -1..1. From the nominal table only."""
    scales = {
        "wheel": v["wheel_speed_max_mmps"], "gyro": math.radians(v["gyro_range_dps"]),
        "accel": v["gravity_mmps2"], "prox": v["tof_range_max_mm"],
        "head": math.radians(v["head_angle_max_deg"]),
        "lift_min": v["lift_height_min_mm"], "lift_span": v["lift_height_max_mm"] - v["lift_height_min_mm"],
        "forward": v["directive_forward_speed_max_mmps"], "turn": v["directive_turn_speed_max_dps"],
    }
    assert all(value > 0.0 for value in scales.values()), "observation scales must be positive"
    assert len(scales) == 9, "scale set changed"
    return scales


def build_observation(state: Any, command: Sequence[float], last_action: Sequence[float],
                      scales: Dict[str, float]) -> Tuple[float, ...]:
    """The policy's input, from one robot_state message. Used in training and in deployment."""
    assert len(command) == 2 and len(last_action) == ACT_DIM, "command and action are pairs"
    obs = (
        state.left_wheel_speed_mmps / scales["wheel"], state.right_wheel_speed_mmps / scales["wheel"],
        state.gyro_radps[0] / scales["gyro"], state.gyro_radps[1] / scales["gyro"],
        state.gyro_radps[2] / scales["gyro"],
        state.accel_mmps2[0] / scales["accel"], state.accel_mmps2[1] / scales["accel"],
        state.accel_mmps2[2] / scales["accel"],
        1.0 if state.cliff_detected else 0.0, state.prox_distance_mm / scales["prox"],
        1.0 if state.prox_found_object else 0.0, state.head_angle_rad / scales["head"],
        (state.lift_height_mm - scales["lift_min"]) / scales["lift_span"],
        command[0] / scales["forward"], command[1] / scales["turn"],
        float(last_action[0]), float(last_action[1]),
    )
    assert len(obs) == OBS_DIM, "observation layout differs from OBS_NAMES"
    return obs


def wheel_speeds_of_action(action: Sequence[float], v: Dict[str, Any]) -> Tuple[float, float]:
    """Map a policy output to two wheel speeds in mm/s, inside the robot's limit."""
    assert len(action) == ACT_DIM, "an action is two numbers"
    top = v["wheel_speed_max_mmps"]
    left = min(max(float(action[0]), -1.0), 1.0) * top
    right = min(max(float(action[1]), -1.0), 1.0) * top
    assert abs(left) <= top and abs(right) <= top, "wheel speed outside the robot's limit"
    return left, right


class VectorTask:
    """One environment. Deterministic for a given seed."""

    def __init__(self, seed: int, config: Optional[TaskConfig] = None,
                 doc: Optional[Dict[str, Any]] = None) -> None:
        assert isinstance(seed, int), "seed must be an int"
        self.config = config if config is not None else TaskConfig()
        self.doc = doc if doc is not None else params.load()
        self.nominal = params.values(self.doc)
        self.scales = observation_scales(self.nominal)
        self.rng = random.Random(seed)
        self.step_s = self.nominal["state_period_ms"] * 0.001
        self.ticks = int(round(self.nominal["state_period_ms"] / self.nominal["control_period_ms"]))
        self.max_steps = int(round(self.config.episode_s / self.step_s))
        self.resample_steps = max(1, int(round(self.config.command_resample_s / self.step_s)))
        assert self.ticks >= 1 and self.max_steps >= 1, "policy step must be at least one control tick"
        self.episodes = 0
        self.sim: Optional[VectorSim] = None
        self.reset()

    # ---- episode set-up -------------------------------------------------

    def _draw_values(self) -> Dict[str, Any]:
        if not self.config.randomise:
            return dict(self.nominal)
        seed = self.rng.randrange(2 ** 31)
        drawn = params.sample(self.doc, seed, kinds=RANDOMISED_KINDS, skip=NOT_RANDOMISED)
        assert drawn["sim_timestep_ms"] == self.nominal["sim_timestep_ms"], "timestep must not be drawn"
        return drawn

    def _draw_command(self) -> Tuple[float, float]:
        """Forward speed in mm/s and turn rate in deg/s, inside the household's directive limits."""
        v = self.nominal
        if self.rng.random() < self.config.zero_command_prob:
            return 0.0, 0.0
        forward = self.rng.uniform(0.0, v["directive_forward_speed_max_mmps"])
        turn = self.rng.uniform(-v["directive_turn_speed_max_dps"], v["directive_turn_speed_max_dps"])
        assert abs(turn) <= v["directive_turn_speed_max_dps"], "turn command outside its limit"
        return forward, turn

    def _draw_start(self, v: Dict[str, Any]) -> Tuple[Tuple[float, float], float]:
        margin = self.config.start_edge_margin_mm
        half_x = max(0.0, 0.5 * v["table_length_mm"] - margin)
        half_y = max(0.0, 0.5 * v["table_width_mm"] - margin)
        start = (self.rng.uniform(-half_x, half_x), self.rng.uniform(-half_y, half_y))
        assert abs(start[0]) <= 0.5 * v["table_length_mm"] and abs(start[1]) <= 0.5 * v["table_width_mm"]
        return start, self.rng.uniform(-180.0, 180.0)

    def reset(self) -> Tuple[float, ...]:
        """Start a new episode with a newly drawn robot, table, pose and command."""
        v = self._draw_values()
        start, heading = self._draw_start(v)
        level = CONTROL_OVERRIDE if self.rng.random() < self.config.override_fraction else CONTROL_DEFAULT
        head = self.rng.uniform(v["head_angle_min_deg"], v["head_angle_max_deg"])
        lift = self.rng.uniform(v["lift_height_min_mm"], v["lift_height_max_mm"])
        self.sim = VectorSim(values=v, control_level=level, start_xy_mm=start, heading_deg=heading,
                             head_deg=head, lift_mm=lift, noise_seed=self.rng.randrange(2 ** 31),
                             body_range_clock=self.config.body_range_clock)
        self.values = v
        self.command = self._draw_command()
        self.last_action = (0.0, 0.0)
        self.steps = 0
        self.episode_return = 0.0
        self.episodes += 1
        assert self.sim.has_control, "a new episode starts with control"
        return self.observation()

    def observation(self) -> Tuple[float, ...]:
        assert self.sim is not None, "reset first"
        return build_observation(self.sim.state, self.command, self.last_action, self.scales)

    # ---- one policy step -------------------------------------------------

    def _reward(self, forward_mmps: float, turn_dps: float, action: Sequence[float]) -> Tuple[float, float, float]:
        cfg = self.config
        err_f = forward_mmps - self.command[0]
        err_t = turn_dps - self.command[1]
        change = math.hypot(action[0] - self.last_action[0], action[1] - self.last_action[1])
        reward = (cfg.weight_forward * math.exp(-((err_f / cfg.sigma_forward_mmps) ** 2))
                  + cfg.weight_turn * math.exp(-((err_t / cfg.sigma_turn_dps) ** 2))
                  - cfg.weight_action_rate * change)
        assert math.isfinite(reward), "reward must be finite"
        return reward, abs(err_f), abs(err_t)

    def _ending(self) -> Tuple[bool, bool, Optional[str]]:
        """(terminated, truncated, reason). A fall is the policy's failure; the other two are not."""
        sim = self.sim
        assert sim is not None, "reset first"
        if sim.sim_truth()["fallen"] or sim.hazards.falling or sim.hazards.picked_up:
            return True, False, END_FALL
        if sim.reactions > 0:
            return False, True, END_REACTION
        if self.steps >= self.max_steps:
            return False, True, END_TIME
        return False, False, None

    def step(self, action: Sequence[float]) -> StepResult:
        """Apply two wheel speeds for one state period and report what the robot then reports."""
        sim = self.sim
        assert sim is not None, "reset first"
        clipped = (min(max(float(action[0]), -1.0), 1.0), min(max(float(action[1]), -1.0), 1.0))
        left, right = wheel_speeds_of_action(clipped, self.values)
        x0, y0, _ = sim.pose_mm()
        yaw0 = sim.heading_total_rad()
        sim.set_wheel_motors(left, right)
        sim.step(self.ticks)
        x1, y1, _ = sim.pose_mm()
        yaw1 = sim.heading_total_rad()
        mid = 0.5 * (yaw0 + yaw1)
        forward = ((x1 - x0) * math.cos(mid) + (y1 - y0) * math.sin(mid)) / self.step_s
        reward, err_f, err_t = self._reward(forward, math.degrees(yaw1 - yaw0) / self.step_s, clipped)
        self.last_action = clipped
        self.steps += 1
        if self.steps % self.resample_steps == 0:
            self.command = self._draw_command()
        terminated, truncated, reason = self._ending()
        if terminated:
            reward -= self.config.fall_penalty
        self.episode_return += reward
        return StepResult(self.observation(), reward, terminated, truncated, reason, err_f, err_t)


def kinematic_action(command: Sequence[float], v: Dict[str, Any]) -> Tuple[float, float]:
    """A hand-written baseline: the no-slip wheel speeds for a command, as an action."""
    assert len(command) == 2, "a command is forward speed and turn rate"
    half = 0.5 * v["tread_spacing_mm"]
    diff = math.radians(command[1]) * half
    top = v["wheel_speed_max_mmps"]
    assert top > 0.0, "wheel speed limit must be positive"
    return (command[0] - diff) / top, (command[0] + diff) / top


def varied_parameters(doc: Dict[str, Any]) -> List[str]:
    """Names the task draws anew each episode."""
    names = [name for name, entry in doc["parameters"].items()
             if entry["kind"] in RANDOMISED_KINDS and "low" in entry and name not in NOT_RANDOMISED]
    assert len(names) > 0, "nothing is randomised"
    return sorted(names)

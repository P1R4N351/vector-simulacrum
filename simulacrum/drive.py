"""The tread drive as the firmware runs it: encoder and speed control..

No MuJoCo here. Two small pieces, each stepped once per 5 ms control tick:

TreadEncoder turns tread travel into encoder counts and the speed the
firmware derives from them (hal_motors.cpp). The encoder has one channel:
it counts edges whichever way the tread turns, and the speed is the number
of new counts over the time between the edges that bound them. No new count
for encoder_zero_speed_ticks ticks reads as zero.

TreadController is wheelController.cpp: motor power from a feedforward line
on the commanded speed plus a proportional and an integral term on the
speed error. Power is never applied against the commanded direction, and a
command at or below wheel_command_stopped_mmps means zero power, so the
robot slows down on the drive's own drag.
"""

from __future__ import annotations

import math
from typing import Any, Dict


class TreadEncoder:
    """Counts of one tread and the unsigned speed measured from them."""

    def __init__(self, v: Dict[str, Any]) -> None:
        self._mm_per_count = float(v["tread_mm_per_encoder_count"])
        self._zero_ticks = int(v["encoder_zero_speed_ticks"])
        self._dt = v["control_period_ms"] * 0.001
        assert self._mm_per_count > 0.0 and self._zero_ticks >= 1, "encoder scale and zero threshold must be positive"
        self.counts = 0
        self.speed_mmps = 0.0
        self.direction = 1.0
        self._drive_direction = 1.0
        self._path_mm = 0.0
        self._last_travel = 0.0
        self._last_edge_s = 0.0
        self._now_s = 0.0
        self._idle_ticks = 0

    def update(self, travel_mm: float, drive: float = 0.0) -> float:
        """Feed the tread's signed travel so far; returns the unsigned measured speed in mm/s."""
        assert math.isfinite(travel_mm), "tread travel must be finite"
        if drive != 0.0:
            self._drive_direction = math.copysign(1.0, drive)
        step = abs(travel_mm - self._last_travel)
        self._last_travel = travel_mm
        start_path = self._path_mm
        self._path_mm += step
        total = int(self._path_mm / self._mm_per_count)
        new = total - self.counts
        if new > 0:
            self.direction = self._drive_direction
            edge_path = total * self._mm_per_count
            edge_s = self._now_s + self._dt * (edge_path - start_path) / step
            span = edge_s - self._last_edge_s
            self.speed_mmps = new * self._mm_per_count / span if span > 0.0 else self.speed_mmps
            self._last_edge_s, self.counts, self._idle_ticks = edge_s, total, 0
        else:
            self._idle_ticks = min(self._idle_ticks + 1, self._zero_ticks)
            if self._idle_ticks >= self._zero_ticks:
                self.speed_mmps = 0.0
        self._now_s += self._dt
        return self.speed_mmps


class TreadController:
    """The firmware's wheel speed loop for one tread. Output is motor power in -1..1."""

    def __init__(self, v: Dict[str, Any]) -> None:
        self._kp, self._ki = v["wheel_kp_power_per_mmps"], v["wheel_ki_power_per_mmps_tick"]
        self._sum_max = v["wheel_error_sum_max"]
        self._slope, self._offset = v["wheel_ff_slope_power_per_mmps"], v["wheel_ff_offset_power"]
        self._low_speed, self._low_slope = v["wheel_ff_low_speed_mmps"], v["wheel_ff_low_slope_power_per_mmps"]
        self._stopped, self._max = v["wheel_command_stopped_mmps"], v["motor_power_max"]
        assert self._max > 0.0 and self._sum_max > 0.0, "power and error-sum limits must be positive"
        assert self._slope > 0.0 and self._low_slope > 0.0, "feedforward slopes must be positive"
        self.error_sum = 0.0
        self.power = 0.0

    def feedforward(self, desired_mmps: float) -> float:
        """The firmware's fitted line of power against commanded speed, with the command's sign."""
        speed = abs(desired_mmps)
        out = self._slope * speed + self._offset if speed > self._low_speed else self._low_slope * speed
        assert out >= 0.0, "feedforward magnitude must not be negative"
        return -out if desired_mmps < 0.0 else out

    def update(self, desired_mmps: float, measured_mmps: float) -> float:
        """One control tick. measured_mmps is the signed speed the firmware believes."""
        assert math.isfinite(desired_mmps) and math.isfinite(measured_mmps), "speeds must be finite"
        error = desired_mmps - measured_mmps
        out = self.feedforward(desired_mmps) + self._kp * error + self._ki * self.error_sum
        if abs(desired_mmps) <= self._stopped:
            self.power, self.error_sum = 0.0, 0.0
            return self.power
        if desired_mmps > 0.0:
            self.power = min(max(out, 0.0), self._max)
        else:
            self.power = min(max(out, -self._max), 0.0)
        if abs(self.power) < self._max:
            self.error_sum = min(max(self.error_sum + error, -self._sum_max), self._sum_max)
        return self.power

    def reset(self) -> None:
        self.error_sum, self.power = 0.0, 0.0


def whole_mmps(speed_mmps: float) -> float:
    """The firmware stores the measured wheel speed in an integer: the fraction is dropped."""
    assert math.isfinite(speed_mmps), "speed must be finite"
    return float(int(speed_mmps))

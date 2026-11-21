"""The forward time-of-flight sensor, as the robot reports it..

No MuJoCo here. The robot layer casts the rays and hands over, for each one,
the distance to what it hit. This module turns those into what the SDK
reports: distance, signal quality, and the three flags.

The sensor sees a cone (tof_fov_deg across), sampled by a fixed fan of rays.
Each ray that hits something returns a signal that falls with distance by the
fitted power law. Signal quality is the mean over the fan, so something that
fills half the cone returns half the signal; the distance is the
signal-weighted mean of the hits. How the surface is turned to the sensor and
what it is made of are not modelled: the recordings do not say.

What the robot does with a reading follows its engine
(engine/components/sensors/proxSensorComponent.cpp, source ENGPROX):

* the distance is clamped to tof_report_min_mm .. tof_report_max_mm;
* between two lift heights the lift is in the beam, and it stays "in view"
  for a short time after it leaves;
* unobstructed means the signal is weak or the distance is at the maximum;
* found_object means none of that, and the robot is not pitched.

With no usable signal the real sensor still reports a distance: on the
recordings a weak, scattered one around tof_weak_distance_mm. That is
reproduced, because a policy trained here will meet it on the robot.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Any, Dict, List, Sequence, Tuple

RINGS = ((0.5, 6), (0.9, 12))  # (fraction of the half-angle, rays on that ring), plus one ray on the axis


@dataclass(frozen=True)
class ProxReading:
    distance_mm: float
    signal_quality: float
    unobstructed: bool
    found_object: bool
    is_lift_in_fov: bool


def ray_directions(v: Dict[str, Any]) -> List[Tuple[float, float, float]]:
    """Unit vectors of the fan in the sensor's frame: z along the axis, x and y across it."""
    half = 0.5 * math.radians(v["tof_fov_deg"])
    assert 0.0 < half < 0.5 * math.pi, "time-of-flight field of view out of range"
    out = [(0.0, 0.0, 1.0)]
    for fraction, count in RINGS:
        polar = fraction * half
        for k in range(count):
            azimuth = 2.0 * math.pi * k / count
            out.append((math.sin(polar) * math.cos(azimuth), math.sin(polar) * math.sin(azimuth), math.cos(polar)))
    assert len(out) == 1 + sum(count for _, count in RINGS), "fan size changed"
    return out


class ProxSensor:
    """Holds the sensor's state between readings (the lift's lingering) and its own noise."""

    def __init__(self, v: Dict[str, Any], noise_seed: int) -> None:
        assert v["tof_report_min_mm"] < v["tof_report_max_mm"] <= v["tof_range_max_mm"], "report limits out of order"
        assert v["tof_lift_occluding_min_mm"] < v["tof_lift_occluding_max_mm"], "lift window is empty"
        assert v["tof_quality_at_100mm"] > 0.0 and v["tof_quality_distance_exponent"] < 0.0, "bad signal law"
        assert v["tof_quality_floor"] < v["tof_quality_min"], "the weak-signal floor must be below the threshold"
        self.v = v
        self.rays = ray_directions(v)
        self._noise = random.Random(noise_seed)
        self._clear_needed = max(1, int(math.ceil(v["tof_lift_clear_ms"] / v["state_period_ms"])))
        self._clear_count = self._clear_needed

    def _lift_in_view(self, lift_height_mm: float) -> bool:
        """In the beam between two heights; still counted in view until it has been out for a while."""
        v = self.v
        if v["tof_lift_occluding_min_mm"] <= lift_height_mm <= v["tof_lift_occluding_max_mm"]:
            self._clear_count = 0
            return True
        self._clear_count = min(self._clear_count + 1, self._clear_needed)
        return self._clear_count < self._clear_needed

    def _signal(self, hits: Sequence[float]) -> Tuple[float, float]:
        """(signal quality, signal-weighted distance in mm) of the fan's hits; distance is 0 with no signal."""
        v = self.v
        assert len(hits) == len(self.rays), "one hit per ray"
        total, weighted = 0.0, 0.0
        for distance_mm in hits:
            if distance_mm < 0.0 or distance_mm > v["tof_range_max_mm"]:
                continue
            near = max(distance_mm, v["tof_report_min_mm"])
            strength = v["tof_quality_at_100mm"] * (near / 100.0) ** v["tof_quality_distance_exponent"]
            total += strength
            weighted += strength * distance_mm
        quality = total / len(self.rays)
        return quality, (weighted / total if total > 0.0 else 0.0)

    def read(self, hits: Sequence[float], lift_height_mm: float, pitch_rad: float) -> ProxReading:
        """One reading. hits: per ray, the distance in mm to what it hit, or -1 for nothing."""
        v = self.v
        assert math.isfinite(lift_height_mm) and math.isfinite(pitch_rad), "lift height and pitch must be finite"
        lift_in_view = self._lift_in_view(lift_height_mm)
        in_beam = v["tof_lift_occluding_min_mm"] <= lift_height_mm <= v["tof_lift_occluding_max_mm"]
        if in_beam:
            quality, raw = v["tof_lift_quality"], v["tof_lift_distance_mm"]
        else:
            quality, raw = self._signal(hits)
        if quality <= v["tof_quality_min"]:
            quality = max(quality, v["tof_quality_floor"])
            raw = self._noise.gauss(v["tof_weak_distance_mm"], v["tof_weak_distance_sd_mm"])
        else:
            raw *= 1.0 + self._noise.gauss(0.0, v["tof_range_std_fraction"])
        limit = math.radians(v["tof_found_pitch_limit_deg"])
        unobstructed = quality <= v["tof_quality_min"] or raw >= v["tof_report_max_mm"]
        found = (not lift_in_view and quality > v["tof_quality_min"] and raw < v["tof_report_max_mm"]
                 and -limit <= pitch_rad <= limit)
        distance = float(round(min(max(raw, v["tof_report_min_mm"]), v["tof_report_max_mm"])))
        return ProxReading(distance, quality, unobstructed, found, lift_in_view)

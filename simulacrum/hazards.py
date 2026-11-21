"""Cliff, fall and pick-up flags, as the robot derives them..

No MuJoCo here: the inputs are the four cliff ranges and the accelerometer
magnitude, once per control tick. The thresholds come from the parameter
table. The pick-up rule is a reduced form of the firmware's (cliff sensors
only); the firmware also uses gyro and accelerometer motion.
"""

from __future__ import annotations

from typing import Any, Dict, List, Sequence


class Hazards:
    """Holds the hazard flags and updates them once per control tick."""

    def __init__(self, v: Dict[str, Any]) -> None:
        assert v["fall_accel_low_mmps2"] < v["fall_accel_high_mmps2"], "fall thresholds out of order"
        assert v["cliff_detect_range_mm"] > v["cliff_sensor_height_mm"], "cliff range inside the sensor height"
        self._range_mm = float(v["cliff_detect_range_mm"])
        self._low = float(v["fall_accel_low_mmps2"])
        self._high = float(v["fall_accel_high_mmps2"])
        self._fall_need = int(round(v["fall_detect_time_ms"] / v["control_period_ms"]))
        self._pickup_need = int(v["pickup_detect_ticks"])
        self._pickup_count = int(v["pickup_cliff_sensor_count"])
        self.cliff: List[bool] = [False, False, False, False]
        self.cliff_any = False
        self.cliff_new = False
        self.falling = False
        self.picked_up = False
        self._fall_ticks = 0
        self._pickup_ticks = 0

    def _update_cliffs(self, ranges_mm: Sequence[float]) -> None:
        assert len(ranges_mm) == 4, "four cliff ranges"
        was_any = self.cliff_any
        for i, reading in enumerate(ranges_mm):
            self.cliff[i] = reading < 0.0 or reading > self._range_mm
        self.cliff_any = any(self.cliff)
        self.cliff_new = self.cliff_any and not was_any

    def _update_fall(self, accel_mag_mmps2: float) -> None:
        """Free fall: low acceleration with a cliff under a sensor, held for the detection time."""
        assert accel_mag_mmps2 >= 0.0, "acceleration magnitude must not be negative"
        if self.falling:
            if accel_mag_mmps2 > self._high:
                self.falling = False
                self._fall_ticks = 0
            return
        if accel_mag_mmps2 < self._low and self.cliff_any:
            self._fall_ticks += 1
        elif accel_mag_mmps2 >= self._high or not self.cliff_any:
            self._fall_ticks = 0
        if self._fall_ticks > self._fall_need:
            self.falling = True

    def _update_pickup(self) -> None:
        count = sum(1 for flag in self.cliff if flag)
        assert 0 <= count <= 4, "cliff count out of range"
        if count >= self._pickup_count:
            self._pickup_ticks = min(self._pickup_ticks + 1, self._pickup_need + 1)
        else:
            self._pickup_ticks = 0
        self.picked_up = self._pickup_ticks > self._pickup_need

    def update(self, ranges_mm: Sequence[float], accel_mag_mmps2: float) -> None:
        self._update_cliffs(ranges_mm)
        self._update_fall(accel_mag_mmps2)
        self._update_pickup()

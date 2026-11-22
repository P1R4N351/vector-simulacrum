"""The fall and pick-up flags, fed known inputs. No MuJoCo.

    python3 tests/run_all.py --only test_hazards

simulacrum/hazards.py derives the robot's `falling` flag from the
accelerometer magnitude and the cliff sensors, once per control tick, as the
firmware does (IMUCPP:547-553). In a simulated drop off the table the flag
depends on how long the tumble's free fall lasts, which is chaotic: on
2026-11-22 the same drop stayed under the threshold for 6 state messages on
MuJoCo 3.11.0, 5 on macOS 4.0.1 and 3 on Linux 4.0.1, which never set it.
So the rule is tested here, on a trace whose answer is known, and
test_sim.Cliff tests the drop itself.
"""

from __future__ import annotations

import unittest

import _common  # noqa: F401  (puts the skill on the import path)
from simulacrum import hazards, params

OVER_EDGE_MM = 100.0  # every sensor reads past cliff_detect_range_mm
ON_TABLE_MM = 10.0  # every sensor sees the surface


class FallFlag(unittest.TestCase):
    def setUp(self) -> None:
        self.v = params.values(params.load())
        self.need = int(round(self.v["fall_detect_time_ms"] / self.v["control_period_ms"]))
        self.low, self.high = self.v["fall_accel_low_mmps2"], self.v["fall_accel_high_mmps2"]
        self.assertGreater(self.need, 1, "the detection time spans several control ticks")
        self.assertGreater(OVER_EDGE_MM, self.v["cliff_detect_range_mm"])
        self.assertLess(ON_TABLE_MM, self.v["cliff_detect_range_mm"])

    def _feed(self, flags: hazards.Hazards, ticks: int, ranges_mm: float, accel: float) -> None:
        assert 0 <= ticks <= 10_000, "tick count out of range"
        for _ in range(ticks):
            flags.update([ranges_mm] * 4, accel)

    def test_low_acceleration_over_a_cliff_sets_falling_only_after_the_detection_time(self) -> None:
        flags = hazards.Hazards(self.v)
        self._feed(flags, self.need, OVER_EDGE_MM, 0.5 * self.low)
        self.assertFalse(flags.falling, "not yet: the count must exceed the detection time")
        self._feed(flags, 1, OVER_EDGE_MM, 0.5 * self.low)
        self.assertTrue(flags.falling)

    def test_no_cliff_means_no_fall_however_low_the_acceleration(self) -> None:
        flags = hazards.Hazards(self.v)
        self._feed(flags, 4 * self.need, ON_TABLE_MM, 0.0)
        self.assertFalse(flags.falling)

    def test_acceleration_between_the_thresholds_holds_the_count_and_a_jolt_resets_it(self) -> None:
        flags = hazards.Hazards(self.v)
        mid = 0.5 * (self.low + self.high)
        self._feed(flags, self.need - 1, OVER_EDGE_MM, 0.5 * self.low)
        self._feed(flags, 3 * self.need, OVER_EDGE_MM, mid)
        self._feed(flags, 2, OVER_EDGE_MM, 0.5 * self.low)
        self.assertTrue(flags.falling, "ticks between the thresholds neither count nor reset")
        reset = hazards.Hazards(self.v)
        self._feed(reset, self.need - 1, OVER_EDGE_MM, 0.5 * self.low)
        self._feed(reset, 1, OVER_EDGE_MM, self.high)
        self._feed(reset, 2, OVER_EDGE_MM, 0.5 * self.low)
        self.assertFalse(reset.falling, "an acceleration at the high threshold starts the count again")

    def test_falling_clears_only_above_the_high_threshold(self) -> None:
        flags = hazards.Hazards(self.v)
        self._feed(flags, self.need + 1, OVER_EDGE_MM, 0.0)
        self.assertTrue(flags.falling)
        self._feed(flags, 5, ON_TABLE_MM, self.high)
        self.assertTrue(flags.falling, "at the threshold, not above it, the flag stays")
        self._feed(flags, 1, ON_TABLE_MM, self.high + 1.0)
        self.assertFalse(flags.falling)


if __name__ == "__main__":
    unittest.main()

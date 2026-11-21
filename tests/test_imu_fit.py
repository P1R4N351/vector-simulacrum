"""The accelerometer's steady offsets, fitted from the recording. Needs numpy, no MuJoCo.."""

from __future__ import annotations

import importlib.util
import math
import unittest

import _common  # noqa: F401
from simulacrum import params, recording

HAVE_NUMPY = importlib.util.find_spec("numpy") is not None
if HAVE_NUMPY:
    from simulacrum import imu_fit

MADE_UP = (0.02, -0.01, 0.03, -400.0, 60.0, 150.0)  # tilt x, tilt y, pitch (rad), then three offsets in mm/s^2


@unittest.skipUnless(HAVE_NUMPY, "numpy is not installed here")
class AccelerometerFit(unittest.TestCase):
    _cached = None

    @classmethod
    def setUpClass(cls) -> None:
        cls.doc = params.load()
        cls.data = imu_fit.stretches(recording.load()[0])

    @property
    def report(self) -> dict:
        """The fit, run once inside a test so that a broken fit fails that test and not the set-up."""
        if AccelerometerFit._cached is None:
            AccelerometerFit._cached = imu_fit.describe()
        return AccelerometerFit._cached

    def test_table_entries_are_the_fitted_values(self) -> None:
        bias, pitch = self.doc["parameters"]["accel_bias_mg"], self.doc["parameters"]["imu_pitch_offset_deg"]
        self.assertEqual((self.report["stretches"], self.report["rows"]), (26, 625))
        for value, fitted in zip(bias["value"], self.report["bias_mg"]):
            self.assertAlmostEqual(value, fitted, delta=0.05)
        self.assertAlmostEqual(pitch["value"], self.report["pitch_deg"], delta=0.005)
        self.assertEqual((bias["provenance"], pitch["provenance"]), ("fitted", "fitted"))
        self.assertLess(self.report["rms_mmps2"], 20.0)
        self.assertAlmostEqual(self.report["table_tilt_deg"], 1.42, delta=0.01)

    def test_the_fit_recovers_made_up_offsets_at_the_recorded_head_angles_and_headings(self) -> None:
        made = [(head, yaw, tuple(imu_fit.predicted(MADE_UP, head, yaw)), rows) for head, yaw, _, rows in self.data]
        found = imu_fit.fit(made)["values"]
        for name, want in zip(imu_fit.NAMES, MADE_UP):
            self.assertAlmostEqual(found[name], want, delta=1e-5 if "rad" in name else 1e-2)
        self.assertLess(imu_fit.fit(made)["rms_mmps2"], 1e-3)

    def test_an_offset_on_x_is_not_mistaken_for_a_pitched_frame(self) -> None:
        only_x = (0.0, 0.0, 0.0, -400.0, 0.0, 0.0)
        made = [(head, yaw, tuple(imu_fit.predicted(only_x, head, yaw)), rows) for head, yaw, _, rows in self.data]
        found = imu_fit.fit(made)["values"]
        self.assertAlmostEqual(math.degrees(found["pitch_rad"]), 0.0, delta=0.001)
        self.assertAlmostEqual(found["bias_x_mmps2"], -400.0, delta=0.01)

    def test_a_head_tilted_up_reads_gravity_on_plus_x(self) -> None:
        level = imu_fit.predicted([0.0] * 6, 0.0, 0.0)
        up = imu_fit.predicted([0.0] * 6, math.radians(30.0), 0.0)
        self.assertAlmostEqual(level[0], 0.0, delta=1e-9)
        self.assertAlmostEqual(level[2], imu_fit.GRAVITY_MMPS2, delta=1e-6)
        self.assertAlmostEqual(up[0], 0.5 * imu_fit.GRAVITY_MMPS2, delta=1e-6)
        nose_down_table = imu_fit.predicted([-0.05, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, 0.0)
        self.assertLess(nose_down_table[0], -400.0)

    def test_leaving_out_any_cluster_of_head_angles_stays_inside_the_ranges(self) -> None:
        bias, pitch = self.doc["parameters"]["accel_bias_mg"], self.doc["parameters"]["imu_pitch_offset_deg"]
        self.assertEqual(sorted(self.report["without"]), ["head near 0 deg", "head near 22 deg", "head near 7 deg"])
        for part in self.report["without"].values():
            self.assertTrue(9 <= part["stretches"] < 26)
            self.assertTrue(pitch["low"] <= part["pitch_deg"] <= pitch["high"])
            for low, value, high, full in zip(bias["low"], part["bias_mg"], bias["high"], self.report["bias_mg"]):
                self.assertTrue(low <= value <= high)
                self.assertLess(abs(value - full), 1.5)

    def test_pitch_and_x_offset_trade_but_their_sum_is_tight_and_the_ranges_cover_the_bootstrap(self) -> None:
        bias, pitch = self.doc["parameters"]["accel_bias_mg"], self.doc["parameters"]["imu_pitch_offset_deg"]
        boot = self.report["bootstrap_95"]
        self.assertLess(self.report["pitch_bias_x_correlation"], -0.95)
        self.assertEqual(boot["draws"], 200)
        self.assertTrue(pitch["low"] <= boot["pitch_deg"][0] < boot["pitch_deg"][1] <= pitch["high"])
        for axis, name in enumerate(("bias_x_mg", "bias_y_mg", "bias_z_mg")):
            self.assertTrue(bias["low"][axis] <= boot[name][0] < boot[name][1] <= bias["high"][axis])
        self.assertLess(boot["level_x_mg"][1] - boot["level_x_mg"][0], 4.0)
        self.assertGreater(boot["bias_x_mg"][1] - boot["bias_x_mg"][0], 3.0)

    def test_reading_on_a_level_surface_is_the_offsets_plus_the_pitched_gravity(self) -> None:
        at_level = imu_fit.level_reading_mg(self.report, 0.0)
        self.assertAlmostEqual(at_level[0], -58.6, delta=0.1)
        self.assertAlmostEqual(at_level[1], self.report["bias_mg"][1], delta=1e-6)
        self.assertAlmostEqual(at_level[2], 22.3, delta=0.1)


if __name__ == "__main__":
    unittest.main()

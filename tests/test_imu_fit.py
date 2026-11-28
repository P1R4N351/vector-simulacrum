"""The accelerometer's steady offsets, fitted from the recording. Needs numpy, no MuJoCo.."""

from __future__ import annotations

import importlib.util
import math
import unittest
from pathlib import Path

import _common  # noqa: F401
from simulacrum import params, recording

HAVE_NUMPY = importlib.util.find_spec("numpy") is not None
if HAVE_NUMPY:
    from simulacrum import imu_fit

MADE_UP = (0.02, -0.01, 0.03, -400.0, 60.0, 150.0)  # tilt x, tilt y, pitch (rad), then three offsets in mm/s^2
SWEEP = Path(__file__).resolve().parent.parent / "evidence" / "head-sweep-unit3-20261120"


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
        scale, bias, pitch = (self.doc["parameters"]["accel_scale"], self.doc["parameters"]["accel_bias_mg"],
                              self.doc["parameters"]["imu_pitch_offset_deg"])
        self.assertEqual((self.report["stretches"], self.report["rows"]), (26, 625))
        self.assertAlmostEqual(scale["value"][0], self.report["scale_x"], delta=0.0005)
        self.assertEqual(scale["value"][1:], [1.0, 1.0])
        for value, fitted in zip(bias["value"], self.report["bias_mg"]):
            self.assertAlmostEqual(value, fitted, delta=0.05)
        self.assertAlmostEqual(pitch["value"], self.report["pitch_deg"], delta=0.005)
        self.assertEqual((scale["provenance"], bias["provenance"], pitch["provenance"]), ("fitted", "fitted", "fitted"))
        self.assertLess(self.report["rms_mmps2"], 20.0)
        self.assertAlmostEqual(self.report["table_tilt_deg"], 1.30, delta=0.05)

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
        scale, bias, pitch = (self.doc["parameters"]["accel_scale"], self.doc["parameters"]["accel_bias_mg"],
                              self.doc["parameters"]["imu_pitch_offset_deg"])
        self.assertEqual(sorted(self.report["without"]), ["head near 0 deg", "head near 22 deg", "head near 7 deg"])
        for part in self.report["without"].values():
            self.assertTrue(9 <= part["stretches"] < 26)
            self.assertTrue(scale["low"][0] <= part["scale_x"] <= scale["high"][0])
            self.assertTrue(pitch["low"] <= part["pitch_deg"] <= pitch["high"])
            for low, value, high, full in zip(bias["low"], part["bias_mg"], bias["high"], self.report["bias_mg"]):
                self.assertTrue(low <= value <= high)
                self.assertLess(abs(value - full), 3.5)

    def test_pitch_and_x_offset_trade_but_their_sum_is_tight_and_the_ranges_cover_the_bootstrap(self) -> None:
        scale, bias, pitch = (self.doc["parameters"]["accel_scale"], self.doc["parameters"]["accel_bias_mg"],
                              self.doc["parameters"]["imu_pitch_offset_deg"])
        boot = self.report["bootstrap_95"]
        self.assertLess(self.report["pitch_bias_x_correlation"], -0.95)
        self.assertEqual(boot["draws"], 200)
        self.assertTrue(scale["low"][0] <= boot["scale_x"][0] < boot["scale_x"][1] <= scale["high"][0])
        self.assertTrue(pitch["low"] <= boot["pitch_deg"][0] < boot["pitch_deg"][1] <= pitch["high"])
        for axis, name in enumerate(("bias_x_mg", "bias_y_mg", "bias_z_mg")):
            self.assertTrue(bias["low"][axis] <= boot[name][0] < boot[name][1] <= bias["high"][axis])
        self.assertLess(boot["level_x_mg"][1] - boot["level_x_mg"][0], 4.0)
        self.assertGreater(boot["bias_x_mg"][1] - boot["bias_x_mg"][0], 3.0)

    def test_reading_on_a_level_surface_is_the_offsets_plus_the_pitched_gravity(self) -> None:
        at_level = imu_fit.level_reading_mg(self.report, 0.0)
        self.assertAlmostEqual(at_level[0], -60.4, delta=0.2)
        self.assertAlmostEqual(at_level[1], self.report["bias_mg"][1], delta=1e-6)
        self.assertAlmostEqual(at_level[2], 22.2, delta=0.1)


@unittest.skipUnless(HAVE_NUMPY, "numpy is not installed here")
class XScaleOnUnit3(unittest.TestCase):
    """The x axis's scale, fitted on spot B of unit 3's head sweeps and scored at spot A."""
    _cached = None

    @classmethod
    def setUpClass(cls) -> None:
        cls.v2 = params.load(model="vector_2")["parameters"]
        cls.spots = imu_fit.sweep_windows(SWEEP)

    @property
    def report(self) -> dict:
        if XScaleOnUnit3._cached is None:
            XScaleOnUnit3._cached = imu_fit.head_sweep(SWEEP)
        return XScaleOnUnit3._cached

    def test_the_gates_keep_the_windows_the_sweeps_authors_counted(self) -> None:
        self.assertEqual(self.report["windows"], {"A": 24, "A90": 6, "B": 24})

    def test_the_2_0_table_holds_the_fitted_values_and_its_ranges_cover_the_bootstrap(self) -> None:
        free, boot = self.report["scale_x_free"], self.report["bootstrap_95"]
        scale, bias, pitch = self.v2["accel_scale"], self.v2["accel_bias_mg"], self.v2["imu_pitch_offset_deg"]
        self.assertEqual({scale["provenance"], bias["provenance"], pitch["provenance"]}, {"fitted"})
        self.assertAlmostEqual(scale["value"][0], free["scale_x"], delta=0.0005)
        self.assertEqual(scale["value"][1:], [1.0, 1.0])
        for value, fitted in zip(bias["value"], free["bias_mg"]):
            self.assertAlmostEqual(value, fitted, delta=0.05)
        self.assertAlmostEqual(pitch["value"], free["pitch_deg"], delta=0.005)
        self.assertTrue(scale["low"][0] <= boot["scale_x"][0] < boot["scale_x"][1] <= scale["high"][0])
        self.assertTrue(pitch["low"] <= boot["pitch_deg"][0] < boot["pitch_deg"][1] <= pitch["high"])
        for axis, name in enumerate(("bias_x_mg", "bias_y_mg", "bias_z_mg")):
            self.assertTrue(bias["low"][axis] <= boot[name][0] < boot[name][1] <= bias["high"][axis])

    def test_freeing_the_x_scale_lowers_the_error_at_the_spot_the_fit_never_saw(self) -> None:
        fixed, free = self.report["scale_1"]["rms_mg"], self.report["scale_x_free"]["rms_mg"]
        self.assertLess(free["A"], fixed["A"] - 0.8)
        self.assertLess(free["A90"], fixed["A90"] - 1.5)
        self.assertGreater(self.report["bootstrap_95"]["scale_x"][0], 1.005)

    def test_a_made_up_scale_is_recovered_at_the_recorded_head_angles(self) -> None:
        made_up = MADE_UP + (0.97,)
        made = [(head, yaw, tuple(imu_fit.predicted(made_up, head, yaw)), 1) for head, yaw, _, _ in self.spots["B"]]
        found = imu_fit.fit(made, free_scale=True)["values"]
        self.assertAlmostEqual(found[imu_fit.SCALE_NAME], 0.97, delta=1e-6)
        self.assertAlmostEqual(found["bias_x_mmps2"], -400.0, delta=0.05)

    def test_rows_with_the_treads_or_the_gyro_moving_and_the_setup_sweep_are_left_out(self) -> None:
        import tempfile
        head = "block,target_deg,visit,rest_s,head_deg,yaw_deg,wl_mm_s,wr_mm_s,gx_rad_s,gy_rad_s,gz_rad_s,ax_mm_s2,ay_mm_s2,az_mm_s2"
        still = "{b},0,1,0.1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,10.0,20.0,9800.0"
        lines = [head] + [still.format(b=b) for b in ("B0cw", "A0cw", "A90ccw")]
        lines += ["B0cw,0,1,0.2,0.0,0.0,5.0,0.0,0.0,0.0,0.0,900.0,20.0,9800.0",
                  "B0cw,0,1,0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.2,900.0,20.0,9800.0",
                  "B0setup,0,1,0.1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,900.0,20.0,9800.0"]
        with tempfile.TemporaryDirectory() as folder:
            (Path(folder) / "samples.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")
            spots = imu_fit.sweep_windows(Path(folder))
        self.assertEqual({spot: len(rows) for spot, rows in spots.items()}, {"A": 1, "A90": 1, "B": 1})
        self.assertEqual(spots["B"][0][2], (10.0, 20.0, 9800.0))

    def test_a_fit_where_some_number_cannot_move_the_readings_is_refused(self) -> None:
        one = self.spots["B"][0]
        with self.assertRaises(AssertionError):
            imu_fit.fit([one] * 8, free_scale=True)

    def test_six_numbers_still_mean_a_scale_of_one(self) -> None:
        self.assertEqual(list(imu_fit.predicted(MADE_UP, 0.4, 0.3)), list(imu_fit.predicted(MADE_UP + (1.0,), 0.4, 0.3)))
        self.assertNotIn(imu_fit.SCALE_NAME, imu_fit.fit(list(self.spots["B"]))["values"])


if __name__ == "__main__":
    unittest.main()

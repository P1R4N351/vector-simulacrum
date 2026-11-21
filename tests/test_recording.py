"""The recording of the real robot, and the table values measured from it. Needs no MuJoCo.."""

from __future__ import annotations

import statistics
import unittest

import _common  # noqa: F401
from simulacrum import params, recording


class RecordingFiles(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.doc = params.load()
        cls.v = params.values(cls.doc)
        cls.facts = recording.describe()

    def test_files_are_the_ones_the_table_cites(self) -> None:
        cited = self.doc["sources"]["REC"]["sha256"]
        self.assertIn(self.facts["table_run_sha256"], cited)
        self.assertIn(self.facts["rest_sha256"], cited)
        self.assertEqual((self.facts["state_rows"], self.facts["command_rows"]), (2700, 51))

    def test_segments_are_the_recorded_sequence(self) -> None:
        _, commands = recording.load()
        segs = recording.segments(commands, self.v)
        self.assertEqual(len([s for s in segs.values() if s.kind == "wheels"]), 10)
        self.assertEqual(len([s for s in segs.values() if s.kind == "head"]), 4)
        self.assertEqual(len([s for s in segs.values() if s.kind == "lift"]), 2)
        spin = segs["spin_ccw_100"]
        self.assertEqual((spin.left, spin.right), (-100.0, 100.0))
        self.assertAlmostEqual(spin.t_stop - spin.t_cmd, 1.56, delta=0.05)
        self.assertEqual((segs["lift_1"].target, segs["lift_2"].target), (92.0, 32.0))

    def test_state_period_is_the_measured_interval(self) -> None:
        interval = self.facts["state_interval_ms"]
        self.assertEqual(interval["n"], 2699)
        self.assertAlmostEqual(interval["median"], 59.8, delta=0.2)
        self.assertAlmostEqual(self.v["state_period_ms"], interval["median"], delta=1.0)
        self.assertAlmostEqual(self.facts["rest_state_interval_ms"]["median"], 59.9, delta=0.2)
        self.assertEqual(self.doc["parameters"]["state_period_ms"]["provenance"], "measured")

    def test_noise_entries_are_the_measured_spread_at_rest(self) -> None:
        rest = self.facts["rest"]
        self.assertEqual((rest["n"], rest["distinct"]), (334, 79))
        self.assertAlmostEqual(self.v["gyro_noise_rms_dps"], statistics.mean(rest["gyro_sd_dps"]), delta=0.001)
        accel_mg = statistics.mean(rest["accel_sd_mmps2"]) / (0.001 * self.v["gravity_mmps2"])
        self.assertAlmostEqual(self.v["accel_noise_rms_mg"], accel_mg, delta=0.02)
        self.assertAlmostEqual(self.v["tof_range_std_fraction"], rest["prox_sd_mm"] / rest["prox_mean_mm"], delta=0.001)
        self.assertLess(max(abs(m) for m in rest["gyro_mean_dps"]), 0.01)
        self.assertAlmostEqual(rest["accel_magnitude_mmps2"] / self.v["gravity_mmps2"], 1.039, delta=0.001)

    def test_cliff_reaction_entries_are_the_one_recorded_event(self) -> None:
        cliff = self.facts["cliff"]
        self.assertEqual(cliff["cliff_rows"], 20)
        self.assertAlmostEqual(cliff["rolled_on_mm"], 1.08, delta=0.05)
        self.assertTrue(cliff["pause_min_s"] <= self.v["cliff_reaction_pause_ms"] / 1000.0 <= cliff["pause_max_s"])
        self.assertAlmostEqual(self.v["cliff_reaction_backup_mm"], cliff["moved_away_mm"], delta=0.5)
        self.assertAlmostEqual(self.v["cliff_reaction_backup_speed_mmps"], cliff["move_speed_mean_mmps"], delta=1.0)

    def test_the_robots_own_fast_turn_stays_well_short_of_the_firmwares_spin_figure(self) -> None:
        v, turn = self.v, self.facts["fast_turn_after_cliff"]
        self.assertEqual(turn["samples"], 3)
        self.assertEqual((turn["wheel_mmps_min"], turn["wheel_mmps_max"]), (210.0, 235.0))
        self.assertAlmostEqual(turn["head_deg"], -10.2, delta=0.1)
        self.assertAlmostEqual(turn["gyro_dps_min"], 275.6, delta=0.1)
        self.assertAlmostEqual(turn["gyro_dps_max"], 301.4, delta=0.1)
        self.assertLess(turn["gyro_dps_max"], v["body_rotation_speed_observed_dps"] - 55.0)
        self.assertTrue(0.48 < turn["ratio_min"] < turn["ratio_max"] < 0.57)

    def test_a_turn_is_read_from_gyro_x_and_z_together(self) -> None:
        tilted = {"gyro": [0.6, 0.0, 0.8]}
        self.assertAlmostEqual(recording.turn_rate_dps(tilted), 57.2958, delta=0.001)
        self.assertAlmostEqual(recording.turn_rate_dps({"gyro": [0.6, 0.0, -0.8]}), -57.2958, delta=0.001)
        self.assertAlmostEqual(recording.turn_rate_dps({"gyro": [0.0, 0.3, 1.0]}), 57.2958, delta=0.001)

    def test_the_gyro_turns_with_the_head(self) -> None:
        frame, entry = self.facts["gyro_frame"], self.doc["parameters"]["imu_pitch_offset_deg"]
        self.assertEqual(frame["n"], 369)
        self.assertAlmostEqual(frame["slope"], 1.0, delta=0.01)
        self.assertLess(frame["head_min_deg"], -15.0)
        self.assertGreater(frame["head_max_deg"], 20.0)
        self.assertAlmostEqual(frame["offset_deg"], -1.06, delta=0.01)
        self.assertTrue(entry["low"] <= frame["offset_deg"] <= entry["high"])

    def test_spin_fraction_entry_is_the_mean_of_the_six_recorded_spins(self) -> None:
        spins = self.facts["spin_ratios"]
        self.assertEqual(sorted(spins), ["spin_ccw_100", "spin_ccw_30", "spin_ccw_60", "spin_cw_100", "spin_cw_30",
                                         "spin_cw_60"])
        self.assertTrue(all(abs(spin["head_deg"] - 22.36) < 0.01 and spin["n"] >= 13 for spin in spins.values()))
        whole = [spin["whole"] for spin in spins.values()]
        self.assertAlmostEqual(statistics.mean(whole), self.v["spin_rate_fraction_measured"], delta=0.005)
        self.assertTrue(0.43 < min(whole) < max(whole) < 0.54)
        self.assertAlmostEqual(statistics.mean(spin["gyro_z"] for spin in spins.values()), 0.463, delta=0.001)
        self.assertTrue(all(spin["whole"] > 1.06 * spin["gyro_z"] for spin in spins.values()))
        for speed in ("30", "60", "100"):
            self.assertGreater(spins["spin_cw_" + speed]["whole"], spins["spin_ccw_" + speed]["whole"])
        self.assertEqual(self.doc["parameters"]["spin_rate_fraction_measured"]["provenance"], "measured")

    def test_sensors_scatter_far_more_in_motion_than_at_rest(self) -> None:
        scatter, rest = self.facts["moving_scatter"], self.facts["rest"]
        self.assertEqual(len(scatter), 6)
        self.assertAlmostEqual(scatter["spin_cw_30"]["gyro_sd_dps"], 1.9, delta=0.1)
        self.assertAlmostEqual(scatter["spin_ccw_100"]["gyro_sd_dps"], 18.8, delta=0.1)
        self.assertAlmostEqual(scatter["spin_ccw_100"]["accel_sd_mg"], 184.8, delta=0.5)
        self.assertGreater(min(s["gyro_sd_dps"] for s in scatter.values()), 40.0 * max(rest["gyro_sd_dps"]))
        self.assertGreater(min(s["accel_sd_mg"] for s in scatter.values()), 10.0 * self.v["accel_noise_rms_mg"])

    def test_gyro_offset_entry_is_the_mean_reading_while_still(self) -> None:
        rest, entry = self.facts["gyro_at_rest"], self.doc["parameters"]["gyro_bias_dps"]
        self.assertEqual((rest["n"], rest["runs"]), (695, 31))
        for value, mean in zip(entry["value"], rest["mean_dps"]):
            self.assertAlmostEqual(value, mean, delta=0.0006)
        self.assertLessEqual(rest["worst_run_mean_dps"], entry["high"][0])
        self.assertGreater(rest["worst_run_mean_dps"], 0.9 * entry["high"][0])
        self.assertEqual(entry["provenance"], "measured")

    def test_reported_wheel_speeds_in_the_recording_are_whole_numbers_signed_by_the_command(self) -> None:
        states, commands = recording.load()
        self.assertTrue(all(row["wl"] == int(row["wl"]) and row["wr"] == int(row["wr"]) for row in states))
        seg = recording.segments(commands, self.v)["spin_ccw_60"]
        during = [row for row in states if seg.t_cmd + 0.5 <= row["t"] <= seg.t_stop]
        self.assertTrue(all(row["wl"] < 0.0 < row["wr"] for row in during))
        self.assertTrue(self.v["wheel_speed_reported_whole_mmps"])


if __name__ == "__main__":
    unittest.main()

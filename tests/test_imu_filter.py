"""The IMU chip's low-pass filter. The filter needs numpy only; the robot tests need MuJoCo.."""

from __future__ import annotations

import math
import statistics
import unittest

import numpy as np

import _common
from simulacrum import imu_filter, params

if _common.HAVE_MUJOCO:
    from simulacrum.robot import VectorSim

TABLE = params.values(params.load())
HALF_POWER = math.sqrt(0.5)
PASS_THROUGH = {"accel_filter_cutoff_hz": 1e6, "gyro_filter_cutoff_hz": 1e6, "imu_filter_second_pole": 0.0}
QUIET = {"gyro_noise_rms_dps": 0.0, "accel_noise_rms_mg": 0.0}
ACCEL_X, GYRO_X, GYRO_Y = 0, 3, 4


def _analog_gain(freq: float, cutoff: float, second_pole: float) -> float:
    lower, higher = imu_filter.corners_hz(cutoff, second_pole)
    return 1.0 / math.sqrt((1.0 + (freq / lower) ** 2) * (1.0 + (freq / higher) ** 2))


def _tone_gain(freq: float, channel: int, second_pole: float, step_s: float) -> float:
    """Steady amplitude of one channel's output for a unit sine on that channel, projected over whole periods."""
    filt = imu_filter.ImuFilter(TABLE["accel_filter_cutoff_hz"], TABLE["gyro_filter_cutoff_hz"], second_pole, step_s)
    n = int(round(4.0 / step_s))
    t = np.arange(n) * step_s
    out = np.empty(n)
    raw = np.zeros(imu_filter.CHANNELS)
    for i in range(n):
        raw[channel] = math.sin(2.0 * math.pi * freq * t[i])
        filt.update(raw)
        out[i] = filt.output[channel]
    tail = int(round(int(2.0 * freq) / freq / step_s))
    assert 0 < tail <= n // 2, "the tail must lie in the second half"
    w = 2.0 * math.pi * freq * t[-tail:]
    return math.hypot(2.0 * float(np.mean(out[-tail:] * np.sin(w))), 2.0 * float(np.mean(out[-tail:] * np.cos(w))))


class Filter(unittest.TestCase):
    def test_the_two_sections_together_are_minus_3_db_at_the_cutoff(self) -> None:
        for q in (0.0, 0.25, 0.5, 1.0):
            for cutoff in (18.65, 40.0):
                self.assertAlmostEqual(_analog_gain(cutoff, cutoff, q), HALF_POWER, delta=1e-12)
        lower, higher = imu_filter.corners_hz(18.65, 0.0)
        self.assertAlmostEqual(lower, 18.65, delta=1e-12)
        self.assertTrue(math.isinf(higher))
        lower, higher = imu_filter.corners_hz(18.65, 1.0)
        self.assertAlmostEqual(lower, higher, delta=1e-9)
        self.assertAlmostEqual(lower / 18.65, 1.0 / math.sqrt(math.sqrt(2.0) - 1.0), delta=1e-12)

    def test_the_discrete_filter_is_minus_3_db_within_one_percent_at_either_timestep(self) -> None:
        for step_s in (0.001, 0.0005):
            for q in (0.0, 1.0):
                gyro = _tone_gain(TABLE["gyro_filter_cutoff_hz"], GYRO_X, q, step_s)
                accel = _tone_gain(TABLE["accel_filter_cutoff_hz"], ACCEL_X, q, step_s)
                self.assertLess(abs(gyro - HALF_POWER), 0.01 * HALF_POWER, (step_s, q, gyro))
                self.assertLess(abs(accel - HALF_POWER), 0.01 * HALF_POWER, (step_s, q, accel))

    def test_a_100_hz_tone_is_cut_as_one_or_two_sections_cut_it(self) -> None:
        cutoff = TABLE["gyro_filter_cutoff_hz"]
        found = {q: _tone_gain(100.0, GYRO_Y, q, 0.001) for q in (0.0, 1.0)}
        for q, gain in found.items():
            self.assertAlmostEqual(gain, _analog_gain(100.0, cutoff, q), delta=0.004)
        self.assertLess(found[1.0], 0.5 * found[0.0])

    def test_a_constant_passes_unchanged_and_each_update_is_counted(self) -> None:
        filt = imu_filter.ImuFilter(40.0, 18.65, 1.0, 0.001)
        start = np.array([10.0, -20.0, 9810.0, 0.1, -0.2, 0.3])
        filt.reset(start)
        for _ in range(50):
            filt.update(start)
        self.assertEqual(filt.updates, 50)
        self.assertTrue(np.allclose(filt.output, start, rtol=0.0, atol=1e-12))
        for _ in range(1000):
            filt.update(2.0 * start)
        self.assertTrue(np.allclose(filt.output, 2.0 * start, rtol=1e-6, atol=0.0))

    def test_the_table_holds_the_firmwares_settings_as_the_datasheet_rates_them(self) -> None:
        doc = params.load()["parameters"]
        gyro, accel, pole = doc["gyro_filter_cutoff_hz"], doc["accel_filter_cutoff_hz"], doc["imu_filter_second_pole"]
        self.assertAlmostEqual(gyro["value"], 74.6 / 4.0, delta=1e-9)
        self.assertAlmostEqual(accel["value"], 80.0 / 2.0, delta=1e-9)
        self.assertTrue(gyro["source"].startswith("The -3 dB point") and "IMUC:134" in gyro["source"])
        self.assertIn("IMUC:132", accel["source"])
        self.assertEqual({gyro["provenance"], accel["provenance"], pole["provenance"]}, {"assumed"})
        self.assertEqual((pole["value"], pole["low"], pole["high"]), (1.0, 0.0, 1.0))


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class InTheRobot(unittest.TestCase):
    def test_accelerometer_reports_signed_hal_counts(self):
        sim = VectorSim(values={**TABLE, **QUIET, "accel_scale": [1]*3, "accel_bias_mg": [0]*3})
        sim._imu_filter.reset(np.array([-25, .12345, 25, 0, 0, 0]))
        sim._sample_imu()
        step = 9810 * TABLE["accel_range_g"] / 32767
        self.assertEqual(sim._imu()[0], (-32768*step, round(123.45/step)*step, 32767*step))

    def test_reported_gyro_has_the_stock_steps_before_residual_bias(self):
        sim = VectorSim(values={**TABLE, **QUIET})
        for target in (-180.013, 0.037, 180.013):
            sim._imu_filter.reset(np.r_[np.zeros(3), np.full(3, math.radians(target))])
            sim._sample_imu()
            for axis, reported in enumerate(sim._imu()[1]):
                step = TABLE["gyro_range_dps"] / 32768 * (0.989 if axis == 2 else 1)
                corrected = math.degrees(reported) - TABLE["gyro_bias_dps"][axis]
                self.assertAlmostEqual(corrected / step, round(corrected / step), places=8)
                self.assertLessEqual(abs(corrected - target), step / 2)

    def test_the_filter_steps_at_every_physics_step_and_starts_settled(self) -> None:
        sim = VectorSim(values=params.values(params.load(), QUIET))
        self.assertEqual(sim._imu_filter.updates, 0)
        sim.step(10)
        self.assertEqual(sim._imu_filter.updates, 10 * sim.d["substeps"])

    def test_the_first_reading_is_the_settled_one_not_a_ramp_from_zero(self) -> None:
        filtered = VectorSim(values=TABLE, noise_seed=4).states[-1]
        raw = VectorSim(values=dict(TABLE, **PASS_THROUGH), noise_seed=4).states[-1]
        for got, want in zip(filtered.accel_mmps2 + filtered.gyro_radps, raw.accel_mmps2 + raw.gyro_radps):
            self.assertAlmostEqual(got, want, delta=1e-6)
        self.assertGreater(filtered.accel_mmps2[2], 0.9 * TABLE["gravity_mmps2"])

    @staticmethod
    def _spin(overrides: dict) -> dict:
        """Unit 3's pose and speed (head 22.5 degrees, 150 mm/s), steady part: mean |roll|, and the turn rate the gyro
        gives (z and x together, by the head's pitch) over the true one from the change of heading."""
        sim = VectorSim(values=params.values(params.load(), dict(QUIET, **overrides)), head_deg=22.5)
        pitch = math.radians(sim.v["imu_pitch_offset_deg"])
        sim.set_wheel_motors(-150.0, 150.0)
        sim.advance(1.5)
        start = sim.heading_total_rad()
        states = [sim.step(1) for _ in range(400)]
        true_rate = (sim.heading_total_rad() - start) / (400 * sim.dt)
        gyro_rate = statistics.mean(s.gyro_radps[2] * math.cos(s.head_angle_rad + pitch)
                                    + s.gyro_radps[0] * math.sin(s.head_angle_rad + pitch) for s in states)
        assert true_rate > 1.0, "the robot did not spin"
        return {"roll": statistics.mean(abs(s.gyro_radps[0] - s.gyro_radps[2] * math.tan(s.head_angle_rad)) for s in states),
                "rate_over_true": gyro_rate / true_rate}

    def test_the_filter_takes_the_treads_vibration_out_of_a_spin_but_not_the_turn(self) -> None:
        """Measured in a GPU host on 2026-11-20 over four spins: filtered, the gyro's turn rate is 0.9927 to 1.0009 of the
        true one; read at one instant per tick it is 0.9871 to 1.0238, which is the aliasing."""
        raw, filtered = self._spin(PASS_THROUGH), self._spin({})
        self.assertLess(filtered["roll"], 0.35 * raw["roll"], (filtered, raw))
        self.assertAlmostEqual(filtered["rate_over_true"], 1.0, delta=0.015)

if __name__ == "__main__":
    unittest.main()

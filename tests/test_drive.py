"""The firmware's encoder arithmetic and tread speed loop. Needs no MuJoCo.."""

from __future__ import annotations

import math
import unittest

import _common  # noqa: F401
from simulacrum import params
from simulacrum.drive import TreadController, TreadEncoder, whole_mmps


def _run_encoder(encoder: TreadEncoder, speed_mmps: float, ticks: int, dt: float = 0.005) -> list:
    travel, readings = encoder._last_travel, []  # carry on from where the tread is
    for _ in range(ticks):
        travel += speed_mmps * dt
        readings.append(encoder.update(travel))
    return readings


class Encoder(unittest.TestCase):
    def setUp(self) -> None:
        self.v = params.values(params.load())

    def test_scale_is_the_firmware_line(self) -> None:
        line = (0.96 * 29.0 * 0.25 * math.pi) / 172.3
        self.assertAlmostEqual(self.v["tread_mm_per_encoder_count"], line, places=6)
        self.assertAlmostEqual(1.0 / self.v["tread_mm_per_encoder_count"], 7.880, places=3)
        self.assertAlmostEqual(self.v["lift_rad_per_encoder_count"], 0.25 * math.pi / 149.7, places=7)
        self.assertAlmostEqual(self.v["head_rad_per_encoder_count"], 0.25 * math.pi / 366.211, places=7)

    def test_counts_follow_travel(self) -> None:
        encoder = TreadEncoder(self.v)
        _run_encoder(encoder, 100.0, 200)
        self.assertEqual(encoder.counts, int(100.0 / self.v["tread_mm_per_encoder_count"]))

    def test_steady_speed_is_measured_exactly(self) -> None:
        for speed in (20.0, 73.0, 220.0):
            encoder = TreadEncoder(self.v)
            self.assertAlmostEqual(_run_encoder(encoder, speed, 100)[-1], speed, places=6, msg=str(speed))

    def test_one_channel_counts_backward_travel_as_travel(self) -> None:
        encoder = TreadEncoder(self.v)
        self.assertAlmostEqual(_run_encoder(encoder, -60.0, 100)[-1], 60.0, places=6)
        self.assertGreater(encoder.counts, 0)

    def test_direction_is_held_with_the_last_edge(self) -> None:
        encoder = TreadEncoder(self.v)
        step = self.v["tread_mm_per_encoder_count"]
        encoder.update(step * 1.5, 0.5)
        encoder.update(step * 1.6, -0.5)
        self.assertEqual(encoder.direction, 1)
        encoder.update(step * 2.5, 0.0)
        self.assertEqual(encoder.direction, -1)

    def test_no_new_count_for_five_ticks_reads_zero(self) -> None:
        encoder = TreadEncoder(self.v)
        _run_encoder(encoder, 100.0, 40)
        held = _run_encoder(encoder, 0.0, 4)
        self.assertAlmostEqual(held[-1], 100.0, places=6, msg="the last speed is held for four ticks")
        self.assertEqual(_run_encoder(encoder, 0.0, 1)[-1], 0.0)

    def test_a_crawl_below_one_count_in_five_ticks_reads_zero(self) -> None:
        slow = 0.5 * self.v["tread_mm_per_encoder_count"] / (5 * 0.005)
        encoder = TreadEncoder(self.v)
        readings = _run_encoder(encoder, slow, 400)
        self.assertGreater(encoder.counts, 5, "the tread did move")
        self.assertGreater(readings.count(0.0), 200, "but most of the time the firmware reads zero")

    def test_measured_speed_is_kept_as_a_whole_number(self) -> None:
        self.assertEqual(whole_mmps(99.9), 99.0)
        self.assertEqual(whole_mmps(-99.9), -99.0)
        self.assertEqual(whole_mmps(0.4), 0.0)


class Controller(unittest.TestCase):
    def setUp(self) -> None:
        self.v = params.values(params.load())
        self.loop = TreadController(self.v)

    def test_feedforward_is_the_firmware_line(self) -> None:
        self.assertAlmostEqual(self.loop.feedforward(100.0), 0.00313 * 100.0 + 0.057, places=9)
        self.assertAlmostEqual(self.loop.feedforward(220.0), 0.7456, places=4)
        self.assertAlmostEqual(self.loop.feedforward(-220.0), -0.7456, places=4)
        self.assertAlmostEqual(self.loop.feedforward(30.0), 0.2, places=6)
        self.assertAlmostEqual(self.loop.feedforward(31.0), 0.00313 * 31.0 + 0.057, places=9)

    def test_power_adds_proportional_and_integral_terms(self) -> None:
        first = self.loop.update(100.0, 80.0)
        self.assertAlmostEqual(first, 0.37 + 0.0005 * 20.0, places=9)
        second = self.loop.update(100.0, 80.0)
        self.assertAlmostEqual(second, 0.37 + 0.0005 * 20.0 + 0.00005 * 20.0, places=9)
        self.assertAlmostEqual(self.loop.error_sum, 40.0, places=9)

    def test_power_is_never_applied_against_the_commanded_direction(self) -> None:
        self.assertEqual(self.loop.update(50.0, 2000.0), 0.0)
        self.assertEqual(TreadController(self.v).update(-50.0, -2000.0), 0.0)

    def test_a_stop_command_means_no_power_and_clears_the_integral(self) -> None:
        for _ in range(50):
            self.loop.update(100.0, 0.0)
        self.assertGreater(self.loop.error_sum, 0.0)
        self.assertEqual(self.loop.update(0.0, 80.0), 0.0)
        self.assertEqual(self.loop.error_sum, 0.0)
        self.assertEqual(self.loop.update(2.0, 80.0), 0.0, "2 mm/s counts as stopped")

    def test_power_and_integral_are_limited(self) -> None:
        for _ in range(4000):
            power = self.loop.update(220.0, 0.0)
        self.assertEqual(power, 1.0)
        self.assertLessEqual(self.loop.error_sum, self.v["wheel_error_sum_max"])
        frozen = self.loop.error_sum
        self.loop.update(220.0, 0.0)
        self.assertEqual(self.loop.error_sum, frozen, "the integral stops growing at full power")


if __name__ == "__main__":
    unittest.main()

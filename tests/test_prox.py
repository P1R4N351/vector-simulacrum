"""The time-of-flight sensor's reporting rules and what the recordings say. No MuJoCo.."""

from __future__ import annotations

import math
import unittest

import _common  # noqa: F401  (puts the skill on the path)
from simulacrum import params, prox, prox_fit

V = params.values(params.load(), {"tof_range_std_fraction": 0.0, "tof_weak_distance_sd_mm": 0.0})
TABLE = params.values(params.load())  # as the table has it, noise included
N = len(prox.ray_directions(V))


def wall(distance_mm: float, rays: int = N) -> list:
    """Hits for something at one distance covering the first `rays` rays of the fan."""
    return [distance_mm] * rays + [-1.0] * (N - rays)


class Fan(unittest.TestCase):
    def test_the_fan_is_unit_rays_inside_the_cone_with_the_axis_first(self) -> None:
        rays = prox.ray_directions(V)
        half = 0.5 * math.radians(V["tof_fov_deg"])
        self.assertEqual(len(rays), 19)
        self.assertEqual(rays[0], (0.0, 0.0, 1.0))
        for ray in rays:
            self.assertAlmostEqual(math.sqrt(sum(c * c for c in ray)), 1.0, places=12)
            self.assertLessEqual(math.acos(min(1.0, ray[2])), half + 1e-12)
        self.assertAlmostEqual(max(math.acos(r[2]) for r in rays), 0.9 * half, places=12)
        self.assertAlmostEqual(sum(r[0] for r in rays), 0.0, places=12, msg="the fan is balanced left to right")
        self.assertAlmostEqual(sum(r[1] for r in rays), 0.0, places=12, msg="and up to down")


class Signal(unittest.TestCase):
    def test_a_wall_filling_the_cone_at_100_mm_gives_the_fitted_quality(self) -> None:
        reading = prox.ProxSensor(V, 0).read(wall(100.0), 80.0, 0.0)
        self.assertAlmostEqual(reading.signal_quality, V["tof_quality_at_100mm"], places=12)
        self.assertEqual(reading.distance_mm, 100.0)
        self.assertTrue(reading.found_object and not reading.unobstructed and not reading.is_lift_in_fov)

    def test_quality_falls_with_distance_and_with_how_much_of_the_cone_is_filled(self) -> None:
        sensor = prox.ProxSensor(V, 0)
        full = sensor.read(wall(100.0), 80.0, 0.0).signal_quality
        self.assertAlmostEqual(sensor.read(wall(200.0), 80.0, 0.0).signal_quality,
                               full * 2.0 ** V["tof_quality_distance_exponent"], places=12)
        nine = sensor.read(wall(100.0, rays=9), 80.0, 0.0)
        self.assertAlmostEqual(nine.signal_quality, full * 9 / N, places=12)
        self.assertEqual(nine.distance_mm, 100.0, "the distance is the hits' own, not thinned by the misses")

    def test_the_distance_is_the_signal_weighted_mean_so_the_nearer_surface_dominates(self) -> None:
        hits = [60.0] * 9 + [300.0] * (N - 9)
        reading = prox.ProxSensor(V, 0).read(hits, 80.0, 0.0)
        self.assertTrue(60.0 <= reading.distance_mm < 70.0, reading.distance_mm)

    def test_a_few_far_hits_are_too_weak_to_count(self) -> None:
        graze = [-1.0] * (N - 3) + [193.0, 281.0, 281.0]  # the table top, as the model's lowest rays reach it
        reading = prox.ProxSensor(V, 0).read(graze, 80.0, 0.0)
        self.assertEqual(reading.signal_quality, V["tof_quality_floor"])
        self.assertEqual(reading.distance_mm, V["tof_weak_distance_mm"])
        self.assertTrue(reading.unobstructed and not reading.found_object)


class Reporting(unittest.TestCase):
    def test_the_distance_is_clamped_to_the_engines_limits(self) -> None:
        strong = dict(V, tof_quality_at_100mm=1000.0)
        self.assertEqual(prox.ProxSensor(strong, 0).read(wall(12.0), 80.0, 0.0).distance_mm, V["tof_report_min_mm"])
        far = prox.ProxSensor(strong, 0).read(wall(900.0), 80.0, 0.0)
        self.assertEqual(far.distance_mm, V["tof_report_max_mm"])
        self.assertTrue(far.unobstructed and not far.found_object, "at the maximum the way counts as clear")
        self.assertFalse(prox.ProxSensor(strong, 0).read(wall(1300.0), 80.0, 0.0).found_object, "past the sensor's own range")

    def test_the_lift_is_in_the_beam_between_two_heights_and_lingers(self) -> None:
        sensor = prox.ProxSensor(V, 0)
        lo, hi = V["tof_lift_occluding_min_mm"], V["tof_lift_occluding_max_mm"]
        for height, expected in ((lo - 0.5, False), (lo, True), (0.5 * (lo + hi), True), (hi, True)):
            fresh = prox.ProxSensor(V, 0)
            self.assertEqual(fresh.read(wall(100.0), height, 0.0).is_lift_in_fov, expected, height)
        blocked = sensor.read(wall(100.0), 45.0, 0.0)
        self.assertEqual((blocked.distance_mm, blocked.signal_quality), (V["tof_report_min_mm"], V["tof_lift_quality"]))
        self.assertFalse(blocked.found_object)
        needed = math.ceil(V["tof_lift_clear_ms"] / V["state_period_ms"])
        after = [sensor.read(wall(100.0), 80.0, 0.0) for _ in range(needed + 1)]
        self.assertEqual([r.is_lift_in_fov for r in after], [True] * (needed - 1) + [False, False])
        self.assertEqual([r.found_object for r in after], [False] * (needed - 1) + [True, True])
        self.assertEqual(after[0].distance_mm, 100.0, "the reading itself is the wall's as soon as the lift is out")

    def test_nothing_is_found_while_the_robot_is_pitched(self) -> None:
        limit = math.radians(V["tof_found_pitch_limit_deg"])
        for pitch, expected in ((0.0, True), (limit, True), (-limit, True), (limit + 0.01, False), (-limit - 0.01, False)):
            self.assertEqual(prox.ProxSensor(V, 0).read(wall(100.0), 80.0, pitch).found_object, expected, pitch)

    def test_noise_comes_from_the_sensors_own_seed(self) -> None:
        noisy = params.values(params.load())
        a = [prox.ProxSensor(noisy, 5).read(wall(-1.0, rays=0), 80.0, 0.0).distance_mm for _ in range(3)]
        b = [prox.ProxSensor(noisy, 5).read(wall(-1.0, rays=0), 80.0, 0.0).distance_mm for _ in range(3)]
        c = prox.ProxSensor(noisy, 6).read(wall(-1.0, rays=0), 80.0, 0.0).distance_mm
        self.assertEqual(a, b)
        self.assertNotEqual(a[0], c)


class Recordings(unittest.TestCase):
    def test_the_fitted_entries_are_what_the_recordings_give(self) -> None:
        got = prox_fit.fit(prox_fit.load_rows(), TABLE)
        self.assertEqual((got["rows"], got["usable_rows"], got["weak_rows"], got["blocked_rows"]), (9929, 4776, 4713, 111))
        for name, places in (("tof_quality_at_100mm", 3), ("tof_quality_distance_exponent", 2), ("tof_quality_floor", 4),
                             ("tof_weak_distance_mm", 6), ("tof_weak_distance_sd_mm", 1), ("tof_lift_quality", 1)):
            self.assertAlmostEqual(got[name], TABLE[name], places=places, msg=name)

    def test_the_recordings_never_leave_the_engines_limits(self) -> None:
        got = prox_fit.fit(prox_fit.load_rows(), V)
        self.assertEqual((got["lowest_distance_mm"], got["highest_distance_mm"]),
                         (V["tof_report_min_mm"], V["tof_report_max_mm"]))


if __name__ == "__main__":
    unittest.main()

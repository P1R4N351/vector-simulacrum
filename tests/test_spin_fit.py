"""The tread contact inset: fitted on unit 1's spins, held out on unit 3's. Needs MuJoCo. Authored-by: CPCS."""

from __future__ import annotations

import importlib.util
import math
import statistics
import unittest

import _common
from simulacrum import params

if _common.HAVE_MUJOCO:
    from simulacrum import spin_fit
HAVE_SCIPY = importlib.util.find_spec("scipy") is not None
if HAVE_SCIPY:
    from simulacrum import camera_spins


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class InsetFit(unittest.TestCase):
    _cached = None

    @classmethod
    def setUpClass(cls) -> None:
        cls.entry = params.load()["parameters"]["tread_contact_inset_mm"]

    @property
    def report(self) -> dict:
        """The fit and the held-out run, made once inside a test so a broken fit fails that test."""
        if InsetFit._cached is None:
            fit = spin_fit.fit(params.values(params.load(model="vector_1")))
            held = spin_fit.held_out(params.values(params.load(model="vector_2")), fit["inset_mm"])
            InsetFit._cached = {"fit": fit, "held_out": held}
        return InsetFit._cached

    def test_the_table_holds_the_fitted_inset_and_its_range_covers_each_spins_fit(self) -> None:
        fit = self.report["fit"]
        self.assertEqual(self.entry["provenance"], "fitted")
        self.assertAlmostEqual(self.entry["value"], fit["inset_mm"], delta=0.05)
        self.assertEqual(len(fit["per_spin_inset_mm"]), 4)
        for label, inset in fit["per_spin_inset_mm"].items():
            self.assertTrue(self.entry["low"] <= inset <= self.entry["high"], label)

    def test_with_no_inset_the_model_spins_too_slowly_and_at_the_fit_it_does_not(self) -> None:
        fit = self.report["fit"]
        self.assertEqual(fit["grid_mm"][0], 0.0)
        self.assertLess(fit["mean_ratio"][0], 0.9)
        at_fit = [r["ratio"] for r in fit["at_fit"]]
        self.assertAlmostEqual(statistics.mean(at_fit), 1.0, delta=0.02)
        self.assertTrue(all(abs(r - 1.0) < 0.06 for r in at_fit), at_fit)

    def test_held_out_unit3_spins_come_closer_to_the_robots_rate(self) -> None:
        held = self.report["held_out"]
        self.assertEqual(len(held["fitted"]), 4)
        for before, after in zip(held["no_inset"], held["fitted"]):
            self.assertEqual(before["label"], after["label"])
            self.assertLess(abs(1.0 - after["ratio"]), abs(1.0 - before["ratio"]), after["label"])
            self.assertGreater(after["ratio"], 0.9, after["label"])

    @unittest.skipUnless(HAVE_SCIPY, "scipy is not installed here; run in CT 930")
    def test_the_inset_keeps_the_turn_centre_within_5_mm_of_where_unit3_turns(self) -> None:
        """Averaged over the fitted inset and 0.04 mm either side: one spin's centre moves by up to 5.7 mm between
        insets 0.02 mm apart (evidence/imu-filter-20261120/inset_scan-output.txt), so one inset alone says little.
        Over that scan the mean of three neighbouring insets sits 2.4 to 4.1 mm ahead of unit 3's centre."""
        measured = statistics.mean(s["centre_x_mm"] for s in camera_spins.turn_centres()["spins"])
        v2 = params.values(params.load(model="vector_2"))
        inset = self.report["fit"]["inset_mm"]
        model = [s["model"]["centre_x_mm"] for s in self.report["held_out"]["fitted"]]
        for step in (-0.04, 0.04):
            model += [s["model"]["centre_x_mm"] for s in
                      spin_fit.compare(spin_fit.HELD_OUT_RECORDING, dict(v2, tread_contact_inset_mm=inset + step))]
        self.assertEqual(len(model), 12)
        self.assertLess(abs(statistics.mean(model) - measured), 5.0, (statistics.mean(model), measured))

    def test_roll_from_the_head_gyro_leaves_out_the_spins_share(self) -> None:
        head = math.radians(22.5)
        roll, pitch = spin_fit._roll_pitch([3.0 * math.sin(head), 0.1, 3.0 * math.cos(head)], head)
        self.assertAlmostEqual(roll, 0.0, delta=1e-12)
        self.assertAlmostEqual(pitch, 0.1, delta=1e-12)

    def test_the_crossing_is_interpolated_and_a_grid_that_never_reaches_one_gives_none(self) -> None:
        self.assertAlmostEqual(spin_fit._crossing([0.0, 1.0, 2.0], [0.8, 0.9, 1.1]), 1.5, delta=1e-12)
        self.assertTrue(math.isnan(spin_fit._crossing([0.0, 1.0], [0.8, 0.9])))


if __name__ == "__main__":
    unittest.main()

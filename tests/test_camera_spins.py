"""Unit 3's turn centre from the camera spins. Needs numpy and scipy, not MuJoCo. Authored-by: CPCS."""

from __future__ import annotations

import csv
import importlib.util
import math
import tempfile
import unittest
from pathlib import Path

import _common  # noqa: F401

HAVE_SCIPY = importlib.util.find_spec("scipy") is not None
if HAVE_SCIPY:
    import numpy as np
    from simulacrum import camera_spins

REASON = "scipy is not installed here; run in CT 930"


@unittest.skipUnless(HAVE_SCIPY, REASON)
class TurnCentres(unittest.TestCase):
    def test_the_four_spins_turn_about_the_points_the_readme_gives(self) -> None:
        found = camera_spins.turn_centres()
        self.assertAlmostEqual(found["focal_px"], 498.4, delta=0.1)
        centres = [s["centre_x_mm"] for s in found["spins"]]
        radii = [s["radius_mm"] for s in found["spins"]]
        for got, want in zip(centres, (-28.9, -26.1, -28.7, -25.4)):
            self.assertAlmostEqual(got, want, delta=0.05)
        for got, want in zip(radii, (8.3, 11.0, 8.7, 11.5)):
            self.assertAlmostEqual(got, want, delta=0.05)
        self.assertTrue(all(abs(s["centre_left_mm"]) < 2.5 for s in found["spins"]))
        self.assertEqual([s["keyframes"] for s in found["spins"]], [8, 9, 9, 9])

    def test_constructed_spins_return_the_turn_centres_they_were_built_with(self) -> None:
        pose = camera_spins.calibration()
        with open(camera_spins.PICKS, newline="", encoding="utf-8") as handle:
            real = [(float(r["u_px"]), float(r["v_px"])) for r in csv.DictReader(handle)]
        middle = camera_spins.intersect(np.array(real), camera_spins.LENS_HEIGHT_MM, pose).mean(axis=0)
        wanted = (-20.0, -25.0, -30.0, -35.0)
        rows = []
        for turn, centre_x in enumerate(wanted, start=1):
            for k in range(9):
                heading = math.radians(10.0 + 32.0 * k)
                forward = np.array([math.cos(heading), math.sin(heading)])
                lens = middle + (camera_spins.LENS_X_MM - centre_x) * forward
                aim = lens + 15.0 * forward
                uv = camera_spins.project(np.array([[*lens, camera_spins.LENS_HEIGHT_MM],
                                                    [*aim, camera_spins.LENS_HEIGHT_MM]]), pose)
                rows.append({"turn": turn, "key": k, "time_s": 0.1 * k, "u_px": uv[0, 0], "v_px": uv[0, 1],
                             "aim_u_px": uv[1, 0], "aim_v_px": uv[1, 1]})
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "picks.csv"
            with open(path, "w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            found = camera_spins.turn_centres(path)
        for spin, want in zip(found["spins"], wanted):
            self.assertAlmostEqual(spin["centre_x_mm"], want, delta=0.05)
            self.assertAlmostEqual(spin["centre_left_mm"], 0.0, delta=0.05)
            self.assertAlmostEqual(spin["radius_mm"], abs(camera_spins.LENS_X_MM - want), delta=0.05)


if __name__ == "__main__":
    unittest.main()
